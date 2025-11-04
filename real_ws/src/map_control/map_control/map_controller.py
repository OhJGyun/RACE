#!/usr/bin/env python3
"""
MAP Controller - Ported from race_stack to ROS2
Original: https://github.com/ForzaETH/race_stack
Minimal changes: only ROS2 compatibility, keeping all original logic intact
"""

import logging
import numpy as np
from .steering_lookup import LookupSteerAngle


class MAP_Controller:
    """This class implements a MAP controller for autonomous driving.
    Input and output topics are managed by the controller manager
    """

    def __init__(self,
                t_clip_min,
                t_clip_max,
                m_l1,
                q_l1,
                speed_lookahead,
                lat_err_coeff,
                acc_scaler_for_steer,
                dec_scaler_for_steer,
                start_scale_speed,
                end_scale_speed,
                downscale_factor,
                speed_lookahead_for_steer,

                prioritize_dyn,
                trailing_gap,
                trailing_p_gain,
                trailing_i_gain,
                trailing_d_gain,
                blind_trailing_speed,

                loop_rate,
                LUT_name,
                state_machine_rate,
                lat_accel_max = 0.0,

                logger_info = logging.info,
                logger_warn = logging.warn
            ):
        # Parameters from manager
        self.t_clip_min = t_clip_min
        self.t_clip_max = t_clip_max
        self.m_l1 = m_l1
        self.q_l1 = q_l1
        self.speed_lookahead = speed_lookahead
        self.lat_err_coeff = lat_err_coeff
        self.acc_scaler_for_steer = acc_scaler_for_steer
        self.dec_scaler_for_steer = dec_scaler_for_steer
        self.start_scale_speed = start_scale_speed
        self.end_scale_speed = end_scale_speed
        self.downscale_factor = downscale_factor
        self.speed_lookahead_for_steer = speed_lookahead_for_steer

        self.prioritize_dyn = prioritize_dyn
        self.trailing_gap = trailing_gap
        self.trailing_p_gain = trailing_p_gain
        self.trailing_i_gain = trailing_i_gain
        self.trailing_d_gain = trailing_d_gain
        self.blind_trailing_speed = blind_trailing_speed

        self.loop_rate = loop_rate
        self.LUT_name = LUT_name
        self.state_machine_rate = state_machine_rate

        # Parameters in the controller
        self.lateral_error_list = [] # list of squared lateral error
        self.curr_steering_angle = 0
        self.idx_nearest_waypoint = None # index of nearest waypoint to car
        self.track_length = None

        self.gap = None
        self.gap_should = None
        self.gap_error = None
        self.gap_actual = None
        self.v_diff = None
        self.i_gap = 0
        self.trailing_command = 2
        self.speed_command = None
        self.curvature_waypoints = 0
        self.d_vs = np.zeros(10)
        self.acceleration_command = 0
        self.lat_accel_max = float(lat_accel_max)

        self.logger_info = logger_info
        self.logger_warn = logger_warn

        self.steer_lookup = LookupSteerAngle(self.LUT_name, logger_info)

    # main loop
    def main_loop(self, state, position_in_map, waypoint_array_in_map, speed_now, opponent, position_in_map_frenet, acc_now, track_length):
        # Updating parameters from manager
        self.state = state
        self.position_in_map = position_in_map
        self.waypoint_array_in_map = waypoint_array_in_map
        self.speed_now = speed_now
        self.opponent = opponent
        self.position_in_map_frenet = position_in_map_frenet
        self.acc_now = acc_now
        self.track_length = track_length
        ## PREPROCESS ##
        # speed vector
        yaw = self.position_in_map[0, 2]
        v = [np.cos(yaw)*self.speed_now, np.sin(yaw)*self.speed_now]

        # calculate lateral error and lateral error norm (lateral_error, self.lateral_error_list, self.lat_e_norm)
        lat_e_norm, lateral_error = self.calc_lateral_error_norm()

        ### LONGITUDINAL CONTROL ###
        self.speed_command = self.calc_speed_command(v, lat_e_norm)

        # POSTPROCESS for acceleration/speed decision
        if self.speed_command is not None:
            speed = np.max([self.speed_command, 0])
            acceleration = 0
            jerk = 0
        else:
            speed = 0
            jerk = 0
            acceleration = 0
            self.logger_warn("[Controller] speed was none")

        ### LATERAL CONTROL ###
        steering_angle = None
        L1_point, L1_distance = self.calc_L1_point(lateral_error)

        if L1_point.any() is not None:
            steering_angle = self.calc_steering_angle(L1_point, L1_distance, yaw, lat_e_norm, v)
        else:
            raise Exception("L1_point is None")

        return speed, acceleration, jerk, steering_angle, L1_point, L1_distance, self.idx_nearest_waypoint

    def calc_steering_angle(self, L1_point, L1_distance, yaw, lat_e_norm, v):
        """
        The purpose of this function is to calculate the steering angle based on the L1 point, desired lateral acceleration and velocity

        Inputs:
            L1_point: point in frenet coordinates at L1 distance in front of the car
            L1_distance: distance of the L1 point to the car
            yaw: yaw angle of the car
            lat_e_norm: normed lateral error
            v : speed vector

        Returns:
            steering_angle: calculated steering angle


        """
        # lookahead for steer (steering delay incorporation by propagating position)
        if self.state == "TRAILING" and (self.opponent is not None):
            speed_la_for_lu = self.speed_now
        else:
            adv_ts_st = self.speed_lookahead_for_steer
            la_position_steer = [self.position_in_map[0, 0] + v[0]*adv_ts_st, self.position_in_map[0, 1] + v[1]*adv_ts_st]
            idx_la_steer = self.nearest_waypoint(la_position_steer, self.waypoint_array_in_map[:, :2])
            speed_la_for_lu = self.waypoint_array_in_map[idx_la_steer, 2]
        speed_for_lu = self.speed_adjust_lat_err(speed_la_for_lu, lat_e_norm)

        L1_vector = np.array([L1_point[0] - self.position_in_map[0, 0], L1_point[1] - self.position_in_map[0, 1]])
        if np.linalg.norm(L1_vector) == 0:
            self.logger_warn("[Controller] norm of L1 vector was 0, eta is set to 0")
            eta = 0
        else:
            eta = np.arcsin(np.dot([-np.sin(yaw), np.cos(yaw)], L1_vector)/np.linalg.norm(L1_vector))

        if L1_distance == 0 or np.sin(eta) == 0:
            lat_acc = 0
            self.logger_warn("[Controller] L1 * np.sin(eta), lat_acc is set to 0")
        else:
            lat_acc = 2*speed_for_lu**2 / L1_distance * np.sin(eta)

        steering_angle = self.steer_lookup.lookup_steer_angle(lat_acc, speed_for_lu)

        # modifying steer based on acceleration
        steering_angle = self.acc_scaling(steering_angle)
        # modifying steer based on speed
        steering_angle = self.speed_steer_scaling(steering_angle, speed_for_lu)

        # modifying steer based on velocity
        steering_angle *= np.clip(1 + (self.speed_now/10), 1, 1.15)

        # limit change of steering angle (rate limiting)
        threshold = 0.55  # max steering change per cycle (rad)
        if abs(steering_angle - self.curr_steering_angle) > threshold:
            # 스티어링 레이트 리미팅 발생 로그 / Log when rate limiting occurs
            clipped_value = np.clip(steering_angle, self.curr_steering_angle - threshold, self.curr_steering_angle + threshold)
            self.logger_info(f"[MAP Controller] STEERING RATE LIMIT: requested={steering_angle:.3f} rad, current={self.curr_steering_angle:.3f} rad, clipped={clipped_value:.3f} rad (max_delta={threshold:.3f})")
            steering_angle = clipped_value
        self.curr_steering_angle = steering_angle
        return steering_angle

    def calc_L1_point(self, lateral_error):
        """
        The purpose of this function is to calculate the L1 point and distance

        Inputs:
            lateral_error: frenet d distance from car's position to nearest waypoint
        Returns:
            L1_point: point in frenet coordinates at L1 distance in front of the car
            L1_distance: distance of the L1 point to the car
        """

        self.idx_nearest_waypoint = self.nearest_waypoint(
            self.position_in_map[0, :2], self.waypoint_array_in_map[:, :2])

        # if all waypoints are equal set self.idx_nearest_waypoint to 0
        if np.isnan(self.idx_nearest_waypoint):
            self.idx_nearest_waypoint = 0

        if len(self.waypoint_array_in_map[self.idx_nearest_waypoint:]) > 2:
            # calculate curvature of global optimizer waypoints
            self.curvature_waypoints = np.mean(abs(self.waypoint_array_in_map[self.idx_nearest_waypoint:,5]))

        # calculate L1 guidance
        L1_distance_raw = self.q_l1 + self.speed_now * self.m_l1

        if abs(self.t_clip_max - self.t_clip_min) < 1e-6:
            L1_distance = self.t_clip_min
            self.logger_info(f"[MAP Controller] L1_D: fixed={L1_distance:.3f}m (t_clip_min==t_clip_max)")
        else:
            # clip lower bound to avoid ultraswerve when far away from mincurv
            lower_bound = max(self.t_clip_min, np.sqrt(2) * lateral_error)
            L1_distance = np.clip(L1_distance_raw, lower_bound, self.t_clip_max)

            # L1 거리 클리핑 로그 / Log when L1 distance is clipped
            if L1_distance != L1_distance_raw:
                self.logger_info(f"[MAP Controller] L1_D CLIPPING: raw={L1_distance_raw:.3f}m -> clipped={L1_distance:.3f}m (bounds=[{lower_bound:.3f}, {self.t_clip_max:.3f}], speed={self.speed_now:.2f}m/s, lat_err={lateral_error:.3f}m)")
            else:
                # 클리핑 없을 때도 주기적으로 L1 정보 로그 (선택적)
                # self.logger_info(f"[MAP Controller] L1_D: {L1_distance:.3f}m (speed={self.speed_now:.2f}m/s)")
                pass

        L1_point = self.waypoint_at_distance_before_car(L1_distance, self.waypoint_array_in_map[:,:2], self.idx_nearest_waypoint)
        return L1_point, L1_distance


    def calc_speed_command(self, v, lat_e_norm):
        """
        The purpose of this function is to isolate the speed calculation from the main control_loop

        Inputs:
            v: speed vector
            lat_e_norm: normed lateral error
            curvature_waypoints: -
        Returns:
            speed_command: calculated and adjusted speed, which can be sent to mux
        """

        # lookahead for speed (speed delay incorporation by propagating position)
        adv_ts_sp = self.speed_lookahead
        la_position = [self.position_in_map[0, 0] + v[0]*adv_ts_sp, self.position_in_map[0, 1] + v[1]*adv_ts_sp]
        idx_la_position = self.nearest_waypoint(la_position, self.waypoint_array_in_map[:, :2])
        global_speed = self.waypoint_array_in_map[idx_la_position, 2]
        if(self.state == "TRAILING" and (self.opponent is not None)): #Trailing controller
            speed_command = self.trailing_controller(global_speed)
        else:
            self.trailing_speed = global_speed
            self.i_gap = 0
            speed_command = global_speed

        speed_command = self.speed_adjust_lat_err(speed_command, lat_e_norm)

        # try:
        #     kappa_here = abs(float(self.waypoint_array_in_map[idx_la_position, 5]))
        # except Exception:
        #     kappa_here = 0
        #     self.logger_warn("[MAP Controller] curvature could not be extracted, set to 0")
        
        # if kappa_here > 1e-6 and self.lat_accel_max > 0.0:
        #     v_max_curv = float(np.sqrt(self.lat_accel_max / max(kappa_here, 1e-6)))
        #     speed_command = float(min(speed_command, v_max_curv))


        return speed_command

    def trailing_controller(self, global_speed):
        """
        Adjust the speed of the ego car to trail the opponent at a fixed distance
        Inputs:
            speed_command: velocity of global raceline
            self.opponent: frenet s position and vs velocity of opponent
            self.position_in_map_frenet: frenet s position and vs veloctz of ego car
        Returns:
            trailing_command: reference velocity for trailing
        """

        self.gap = (self.opponent[0] - self.position_in_map_frenet[0])%self.track_length # gap to opponent
        self.gap_actual = self.gap
        self.gap_should = self.trailing_gap
        self.gap_error = self.gap_should - self.gap_actual
        self.v_diff =  self.position_in_map_frenet[2] - self.opponent[2]
        self.i_gap = np.clip(self.i_gap + self.gap_error/self.loop_rate, -10, 10)

        p_value = self.gap_error * self.trailing_p_gain
        d_value = self.v_diff * self.trailing_d_gain
        i_value = self.i_gap * self.trailing_i_gain


        self.trailing_command = np.clip(self.opponent[2] - p_value - i_value - d_value, 0, global_speed)
        if not self.opponent[4] and self.gap_actual > self.gap_should:
            self.trailing_command = max(self.blind_trailing_speed, self.trailing_command)

        return self.trailing_command


    def distance(self, point1, point2):
        return np.linalg.norm(point2 - point1)

    def acc_scaling(self, steer):
        """
        Steer scaling based on acceleration
        increase steer when accelerating
        decrease steer when decelerating

        Returns:
            steer: scaled steering angle based on acceleration
        """
        if np.mean(self.acc_now) >= 1:
            steer *= self.acc_scaler_for_steer
        elif np.mean(self.acc_now) <= -1:
            steer *= self.dec_scaler_for_steer
        return steer

    def speed_steer_scaling(self, steer, speed):
        """
        Steer scaling based on speed
        decrease steer when driving fast

        Returns:
            steer: scaled steering angle based on speed
        """
        speed_diff = max(0.1,self.end_scale_speed-self.start_scale_speed) # to prevent division by zero
        factor = 1 - np.clip((speed - self.start_scale_speed)/(speed_diff), 0.0, 1.0) * self.downscale_factor
        steer *= factor
        return steer

    def calc_lateral_error_norm(self):
        """
        Calculates lateral error

        Returns:
            lat_e_norm: normalization of the lateral error
            lateral_error: distance from car's position to nearest waypoint
        """
        # DONE rename function and adapt
        lateral_error = abs(self.position_in_map_frenet[1]) # frenet coordinates d

        max_lat_e = 0.5
        min_lat_e = 0.
        lat_e_clip = np.clip(lateral_error, a_min=min_lat_e, a_max=max_lat_e)
        lat_e_norm = 0.5 * ((lat_e_clip - min_lat_e) / (max_lat_e - min_lat_e))
        return lat_e_norm, lateral_error

    def speed_adjust_lat_err(self, global_speed, lat_e_norm):
        """
        Reduce speed from the global_speed based on the lateral error
        and curvature of the track. lat_e_coeff scales the speed reduction:
        lat_e_coeff = 0: no account for lateral error
        lat_e_coaff = 1: maximum accounting

        Returns:
            global_speed: the speed we want to follow
        """
        # scaling down global speed with lateral error and curvature
        lat_e_coeff = self.lat_err_coeff # must be in [0, 1]
        lat_e_norm *= 2
        curv = np.clip(2*(np.mean(self.curvature_waypoints)/0.8) - 2, a_min = 0, a_max = 1) # 0.8 ca. max curvature mean

        global_speed *= (1 - lat_e_coeff + lat_e_coeff*np.exp(-lat_e_norm*curv))
        return global_speed

    def speed_adjust_heading(self, speed_command):
        """
        Reduce speed from the global_speed based on the heading error.
        If the difference between the map heading and the actual heading
        is larger than 20 degrees, the speed gets scaled down linearly up to 0.5x

        Returns:
            global_speed: the speed we want to follow
        """

        heading = self.position_in_map[0,2]
        map_heading = self.waypoint_array_in_map[self.idx_nearest_waypoint, 6]
        if abs(heading - map_heading) > np.pi: # resolves wrapping issues
            heading_error = 2*np.pi - abs(heading- map_heading)
        else:
            heading_error = abs(heading - map_heading)

        if heading_error < np.pi/9: # 20 degrees error is okay
            return speed_command
        elif heading_error < np.pi/2:
            scaler = 1 - 0.5* heading_error/(np.pi/2) # scale linearly to 0.5x
        else:
            scaler = 0.5
        self.logger_info(f"[MAP Controller] heading error decreasing velocity by {scaler}")
        return speed_command * scaler

    def nearest_waypoint(self, position, waypoints):
        """
        Calculates index of nearest waypoint to the car

        Returns:
            index of nearest waypoint to the car
        """
        position_array = np.array([position]*len(waypoints))
        distances_to_position = np.linalg.norm(abs(position_array - waypoints), axis=1)
        return np.argmin(distances_to_position)

        # Alternative (commented): Sliding-window nearest waypoint around previous index (±K)
        # -------------------------------------------------------------------------------
        # This version reduces computation by searching only in a local window
        # around the last nearest index. Fall back to full scan when last index is
        # unknown or when a large jump is detected.
        #
        # Example usage:
        #   last_idx = getattr(self, 'idx_nearest_waypoint', None)
        #   K = 20  # window half-size (tune by speed/loop_rate/spacing)
        #   n = len(waypoints)
        #   if last_idx is None or n == 0:
        #       position_array = np.array([position]*n)
        #       distances = np.linalg.norm(position_array - waypoints, axis=1)
        #       return int(np.argmin(distances))
        #   center = int(last_idx) % n
        #   lo = max(0, center - K)
        #   hi = min(n, center + K + 1)
        #   window = waypoints[lo:hi]
        #   dists = np.linalg.norm(window - position, axis=1)
        #   return lo + int(np.argmin(dists))

    def waypoint_at_distance_before_car(self, distance, waypoints, idx_waypoint_behind_car):
        """
        Calculates the waypoint at a certain frenet distance in front of the car

        Returns:
            waypoint as numpy array at a ceratin distance in front of the car
        """
        if distance is None:
            distance = self.t_clip_min

        if len(waypoints) == 0:
            raise ValueError("Waypoints array is empty")

        # race_stack-style: assume fixed waypoint spacing (~0.1 m) and step indices forward
        # Note: This replaces the s-based selection below for simplicity and easier debugging.
        # current_index = int(idx_waypoint_behind_car)
        # waypoints_distance = 0.1
        # d_index = int(distance / waypoints_distance + 0.5)
        # target_index = min(len(waypoints) - 1, current_index + d_index)
        # return np.array(waypoints[target_index])

        # Previous RACE implementation using s-based lookup (kept for reference):
        current_index = int(idx_waypoint_behind_car) % len(waypoints)
        if self.track_length <= 0.0:
            # fallback: use simple index wrap with estimated spacing
            approx_spacing = np.maximum(1e-3, np.linalg.norm(waypoints[(current_index + 1) % len(waypoints), :2] - waypoints[current_index, :2]))
            steps = int(round(distance / approx_spacing))
            target_index = (current_index + steps) % len(waypoints)
            return np.array(waypoints[target_index])
        
        current_s = self.waypoint_array_in_map[current_index, 4]
        target_s = current_s + distance
        s_column = self.waypoint_array_in_map[:, 4]
        extended_s = np.concatenate([s_column, s_column[1:] + self.track_length])
        extended_waypoints = np.vstack([waypoints, waypoints[1:]])
        target_index = np.searchsorted(extended_s, target_s, side='left')
        target_index = min(target_index, len(extended_waypoints) - 1)

        # DEBUG: Log lookahead index difference
        final_index = target_index % len(waypoints)
        idx_diff = final_index - current_index
        if idx_diff < 0:
            idx_diff += len(waypoints)  # Handle wraparound

        if hasattr(self, '_debug_ld_counter'):
            self._debug_ld_counter += 1
        else:
            self._debug_ld_counter = 0

        if self._debug_ld_counter % 20 == 0:  # Log every 20 calls (~0.5s at 40Hz)
            self.logger_info(
                f"[LD] curr_idx={current_index}, target_idx={final_index}, "
                f"idx_diff={idx_diff}, LD={distance:.2f}m"
            )

        return np.array(extended_waypoints[target_index % len(waypoints)])
