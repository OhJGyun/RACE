#!/usr/bin/env python3
from casadi import *
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import csv
import os
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
from mpcc import MPC
# from osuf1_common.msg import MPC_metadata, MPC_trajectory, MPC_prediction  # TODO: Add custom messages

class MPCKinematicNode(Node):
    def __init__(self):
        super().__init__('mpc_node')

        # Declare all parameters with default values
        self.declare_parameter('dT', 0.2)
        self.declare_parameter('mpc_steps_N', 20)
        self.declare_parameter('vehicle_L', 0.325)
        self.declare_parameter('mpc_max_steering', 0.523)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('p_min', 0.0)
        self.declare_parameter('p_max', 3.0)
        self.declare_parameter('x_min', -200.0)
        self.declare_parameter('x_max', 200.0)
        self.declare_parameter('y_min', -200.0)
        self.declare_parameter('y_max', 200.0)
        self.declare_parameter('psi_min', -1000.0)
        self.declare_parameter('psi_max', 1000.0)
        self.declare_parameter('s_min', 0.0)
        self.declare_parameter('s_max', 200.0)
        self.declare_parameter('d_v_bound', 2.0)
        self.declare_parameter('d_theta_bound', 0.5)
        self.declare_parameter('d_p_bound', 2.0)
        self.declare_parameter('mpc_ref_vel', 2.0)
        self.declare_parameter('mpc_w_cte', 750.0)
        self.declare_parameter('mpc_w_s', 0.0)
        self.declare_parameter('mpc_w_lag', 750.0)
        self.declare_parameter('mpc_w_vel', 0.75)
        self.declare_parameter('mpc_w_delta', 50.0)
        self.declare_parameter('mpc_w_p', 5.0)
        self.declare_parameter('mpc_w_accel', 4.0)
        self.declare_parameter('mpc_w_delta_d', 750.0)
        self.declare_parameter('mpc_w_delta_p', 0.0)
        self.declare_parameter('spline_poly_order', 3)
        self.declare_parameter('integration_mode', 'Euler')
        self.declare_parameter('ipopt_verbose', True)
        self.declare_parameter('path_folder_name', 'kelley')
        self.declare_parameter('controller_freq', 20.0)
        self.declare_parameter('goal_threshold', 0.75)
        self.declare_parameter('car_width', 0.30)
        self.declare_parameter('inflation_factor', 0.9)
        self.declare_parameter('lag_time', 0.1)
        self.declare_parameter('debug_mode', True)
        self.declare_parameter('delay_mode', True)
        self.declare_parameter('throttle_mode', True)
        self.declare_parameter('localized_pose_topic_name', '/pf/viz/inferred_pose')
        self.declare_parameter('cmd_vel_topic_name', '/drive')
        self.declare_parameter('odom_topic_name', '/ego_racecar/odom')
        self.declare_parameter('goal_topic_name', '/goal_pose')
        self.declare_parameter('mpc_prediction_topic', 'mpc_prediction')
        self.declare_parameter('mpc_metadata_topic', 'mpc_metadata')
        self.declare_parameter('car_frame', 'base_link')
        self.declare_parameter('arc_length_min_dist_tol', 0.05)

        # Get all parameters
        self.param = {
            'dT': self.get_parameter('dT').value,
            'N': self.get_parameter('mpc_steps_N').value,
            'L': self.get_parameter('vehicle_L').value,
            'theta_max': self.get_parameter('mpc_max_steering').value,
            'v_max': self.get_parameter('max_speed').value,
            'p_min': self.get_parameter('p_min').value,
            'p_max': self.get_parameter('p_max').value,
            'x_min': self.get_parameter('x_min').value,
            'x_max': self.get_parameter('x_max').value,
            'y_min': self.get_parameter('y_min').value,
            'y_max': self.get_parameter('y_max').value,
            'psi_min': self.get_parameter('psi_min').value,
            'psi_max': self.get_parameter('psi_max').value,
            's_min': self.get_parameter('s_min').value,
            's_max': self.get_parameter('s_max').value,
            'd_v_bound': self.get_parameter('d_v_bound').value,
            'd_theta_bound': self.get_parameter('d_theta_bound').value,
            'd_p_bound': self.get_parameter('d_p_bound').value,
            'ref_vel': self.get_parameter('mpc_ref_vel').value,
            'mpc_w_cte': self.get_parameter('mpc_w_cte').value,
            'mpc_w_s': self.get_parameter('mpc_w_s').value,
            'mpc_w_lag': self.get_parameter('mpc_w_lag').value,
            'mpc_w_vel': self.get_parameter('mpc_w_vel').value,
            'mpc_w_delta': self.get_parameter('mpc_w_delta').value,
            'mpc_w_p': self.get_parameter('mpc_w_p').value,
            'mpc_w_accel': self.get_parameter('mpc_w_accel').value,
            'mpc_w_delta_d': self.get_parameter('mpc_w_delta_d').value,
            'mpc_w_delta_p': self.get_parameter('mpc_w_delta_p').value,
            'spline_poly_order': self.get_parameter('spline_poly_order').value,
            'INTEGRATION_MODE': self.get_parameter('integration_mode').value,
            'ipopt_verbose': self.get_parameter('ipopt_verbose').value
        }

        dirname = os.path.dirname(__file__)
        path_folder_name = self.get_parameter('path_folder_name').value
        self.CENTER_TRACK_FILENAME = os.path.join(dirname, path_folder_name + '/centerline_waypoints.csv')
        self.CENTER_DERIVATIVE_FILENAME = os.path.join(dirname, path_folder_name + '/center_spline_derivatives.csv')
        self.RIGHT_TRACK_FILENAME = os.path.join(dirname, path_folder_name + '/right_waypoints.csv')
        self.LEFT_TRACK_FILENAME = os.path.join(dirname, path_folder_name + '/left_waypoints.csv')
        self.CONTROLLER_FREQ = self.get_parameter('controller_freq').value
        self.GOAL_THRESHOLD = self.get_parameter('goal_threshold').value
        self.CAR_WIDTH = self.get_parameter('car_width').value
        self.INFLATION_FACTOR = self.get_parameter('inflation_factor').value
        self.LAG_TIME = self.get_parameter('lag_time').value

        self.DEBUG_MODE = self.get_parameter('debug_mode').value
        self.DELAY_MODE = self.get_parameter('delay_mode').value
        self.THROTTLE_MODE = self.get_parameter('throttle_mode').value

        # Topic name related parameters
        pose_topic = self.get_parameter('localized_pose_topic_name').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic_name').value
        odom_topic = self.get_parameter('odom_topic_name').value
        goal_topic = self.get_parameter('goal_topic_name').value
        prediction_pub_topic = self.get_parameter('mpc_prediction_topic').value
        meta_pub_topic = self.get_parameter('mpc_metadata_topic').value
        self.car_frame = self.get_parameter('car_frame').value

        # Path related variables
        self.path_points = None
        self.center_lane = None
        self.center_point_angles = None
        self.center_lut_x, self.center_lut_y = None, None
        self.center_lut_dx, self.center_lut_dy = None, None
        self.right_lut_x, self.right_lut_y = None, None
        self.left_lut_x, self.left_lut_y = None, None
        self.element_arc_lengths = None
        self.element_arc_lengths_orig = None

        # Plot related variables
        self.current_time = 0
        self.t_plot = []
        self.v_plot = []
        self.steering_plot = []
        self.cte_plot = []
        self.time_plot = []

        # Minimum distance search related variables
        self.ARC_LENGTH_MIN_DIST_TOL = self.get_parameter('arc_length_min_dist_tol').value

        # QoS profile for real-time performance
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, cmd_vel_topic, 10)
        self.mpc_trajectory_pub = self.create_publisher(Path, '/mpc_trajectory', 10)
        self.center_path_pub = self.create_publisher(Path, '/center_path', 10)
        self.right_path_pub = self.create_publisher(Path, '/right_path', 10)
        self.left_path_pub = self.create_publisher(Path, '/left_path', 10)
        self.center_tangent_pub = self.create_publisher(PoseStamped, '/center_tangent', 10)
        self.path_boundary_pub = self.create_publisher(MarkerArray, '/boundary_marker', 10)
        # self.prediction_pub = self.create_publisher(MPC_trajectory, prediction_pub_topic, 1)
        # self.meta_pub = self.create_publisher(MPC_metadata, meta_pub_topic, 1)

        # MPC related initializations
        self.mpc = MPC()
        self.mpc.boundary_pub = self.path_boundary_pub
        self.initialize_MPC()
        self.current_pos_x, self.current_pos_y, self.current_yaw, self.current_s = 0.0, 0.0, 0.0, 0.0
        self.current_pose = None
        self.current_vel_odom = 0.0
        self.projected_vel = 0.0
        self.steering_angle = 0.0

        # Goal status related variables
        self.goal_pos = None
        self.goal_reached = False
        self.goal_received = False

        # Lap counter variables
        self.lap_count = 0
        self.max_laps = 20
        self.prev_s = 0.0
        self.racing_started = False

        # Subscribers
        self.pose_sub = self.create_subscription(Odometry, pose_topic, self.pf_pose_callback, qos_profile)
        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goalCB, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomCB, qos_profile)

        # Timer callback function for the control loop
        timer_period = 1.0 / self.CONTROLLER_FREQ
        self.timer = self.create_timer(timer_period, self.controlLoopCB)

        # Timer for publishing track boundaries periodically
        self.path_publish_timer = self.create_timer(1.0, self.publish_track_boundaries)

    def initialize_MPC(self):
        self.preprocess_track_data()
        self.param['s_max'] = self.element_arc_lengths[-1]

        # Print key parameters for verification
        self.get_logger().info("=" * 60)
        self.get_logger().info("MPC Parameters Loaded:")
        self.get_logger().info(f"  Vehicle wheelbase (L): {self.param['L']:.5f} m")
        self.get_logger().info(f"  Max speed: {self.param['v_max']:.2f} m/s")
        self.get_logger().info(f"  Reference velocity: {self.param['ref_vel']:.2f} m/s")
        self.get_logger().info(f"  Max steering angle: {self.param['theta_max']:.4f} rad ({np.rad2deg(self.param['theta_max']):.2f} deg)")
        self.get_logger().info(f"  Time step (dT): {self.param['dT']:.2f} s")
        self.get_logger().info(f"  Prediction horizon (N): {self.param['N']}")
        self.get_logger().info(f"  Track arc length: {self.param['s_max']:.2f} m")
        self.get_logger().info(f"  Weight - CTE: {self.param['mpc_w_cte']:.1f}")
        self.get_logger().info(f"  Weight - Lag: {self.param['mpc_w_lag']:.1f}")
        self.get_logger().info(f"  Weight - Vel: {self.param['mpc_w_vel']:.1f}")
        self.get_logger().info(f"  Weight - Progress: {self.param['mpc_w_p']:.1f}")
        self.get_logger().info(f"  Weight - Delta: {self.param['mpc_w_delta']:.1f}")
        self.get_logger().info(f"  Weight - Delta_d: {self.param['mpc_w_delta_d']:.1f}")
        self.get_logger().info("=" * 60)

        self.mpc.set_initial_params(self.param)
        self.mpc.set_track_data(self.center_lut_x, self.center_lut_y, self.center_lut_dx, self.center_lut_dy,
                                self.right_lut_x, self.right_lut_y, self.left_lut_x, self.left_lut_y,
                                self.element_arc_lengths, self.element_arc_lengths_orig[-1])

        self.mpc.setup_MPC()

    def create_header(self, frame_id):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def find_nearest_index(self, car_pos):
        distances_array = np.linalg.norm(self.center_lane - car_pos, axis=1)
        min_dist_idx = np.argmin(distances_array)
        return min_dist_idx, distances_array[min_dist_idx]

    def heading(self, yaw):
        r = R.from_euler('xyz', [0, 0, yaw])
        q = r.as_quat()  # returns [x, y, z, w]
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def quaternion_to_euler_yaw(self, orientation):
        r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        euler = r.as_euler('xyz')
        return euler[2]  # yaw is the z-axis rotation

    def read_waypoints_array_from_csv(self, filename):
        '''read waypoints from given csv file and return the data in the form of numpy array'''
        if filename == '':
            raise ValueError('No any file path for waypoints file')
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f, delimiter=',')]
        path_points = np.array([[float(point[0]), float(point[1])] for point in path_points])
        return path_points

    def pf_pose_callback(self, msg):
        '''acquire estimated pose of car from particle filter or odometry'''
        # Handle both PoseStamped and Odometry messages
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
            # Odometry message
            pose = msg.pose.pose
        else:
            # PoseStamped message
            pose = msg.pose

        self.current_pos_x = pose.position.x
        self.current_pos_y = pose.position.y
        self.current_yaw = self.quaternion_to_euler_yaw(pose.orientation)
        self.current_pose = [self.current_pos_x, self.current_pos_y, self.current_yaw]

    def odomCB(self, msg):
        '''Get odometry data especially velocity from the car'''
        self.current_vel_odom = msg.twist.twist.linear.x

    def goalCB(self, msg):
        '''Get goal pose from the user - starts racing for max_laps'''
        if not self.racing_started:
            self.racing_started = True
            self.lap_count = 0
            self.get_logger().info(f"Racing started! Target: {self.max_laps} laps")
        self.goal_pos = msg.pose.position
        self.goal_received = True
        self.goal_reached = False

    def publish_track_boundaries(self):
        '''Periodically publish track boundaries for visualization'''
        if hasattr(self, 'center_lane') and hasattr(self, 'right_lane') and hasattr(self, 'left_lane'):
            self.publish_path(self.center_lane, self.center_path_pub)
            self.publish_path(self.right_lane, self.right_path_pub)
            self.publish_path(self.left_lane, self.left_path_pub)

    def publish_path(self, waypoints, publisher):
        # Visualize path derived from the given waypoints in the path
        path = Path()
        path.header = self.create_header('map')
        path.poses = []
        for point in waypoints:
            tempPose = PoseStamped()
            tempPose.header = path.header
            tempPose.pose.position.x = point[0]
            tempPose.pose.position.y = point[1]
            tempPose.pose.orientation.w = 1.0
            path.poses.append(tempPose)
        publisher.publish(path)

    def get_interpolated_path(self, pts, arc_lengths_arr, smooth_value=0.1, scale=2, derivative_order=0):
        # tck represents vector of knots, the B-spline coefficients, and the degree of the spline.
        tck, u = splprep(pts.T, u=arc_lengths_arr, s=smooth_value, per=1)
        u_new = np.linspace(u.min(), u.max(), len(pts) * scale)
        x_new, y_new = splev(u_new, tck, der=derivative_order)
        interp_points = np.concatenate((x_new.reshape((-1, 1)), y_new.reshape((-1, 1))), axis=1)
        return interp_points, tck

    def get_interpolated_path_casadi(self, label_x, label_y, pts, arc_lengths_arr):
        u = arc_lengths_arr
        V_X = pts[:, 0]
        V_Y = pts[:, 1]
        lut_x = interpolant(label_x, 'bspline', [u], V_X)
        lut_y = interpolant(label_y, 'bspline', [u], V_Y)
        return lut_x, lut_y

    def get_arc_lengths(self, waypoints):
        d = np.diff(waypoints, axis=0)
        consecutive_diff = np.sqrt(np.sum(np.power(d, 2), axis=1))
        dists_cum = np.cumsum(consecutive_diff)
        dists_cum = np.insert(dists_cum, 0, 0.0)
        return dists_cum

    def inflate_track_boundaries(self, center_lane, side_lane, car_width=0.325, inflation_factor=1.2):
        for idx in range(len(center_lane)):
            lane_vector = side_lane[idx, :] - center_lane[idx, :]
            side_track_width = np.linalg.norm(lane_vector)
            side_unit_vector = lane_vector / side_track_width
            side_lane[idx, :] = center_lane[idx, :] + side_unit_vector * (
                    side_track_width - car_width * inflation_factor)
        return side_lane

    def preprocess_track_data(self):
        center_lane = self.read_waypoints_array_from_csv(self.CENTER_TRACK_FILENAME)
        center_derivative_data = self.read_waypoints_array_from_csv(self.CENTER_DERIVATIVE_FILENAME)
        right_lane = self.read_waypoints_array_from_csv(self.RIGHT_TRACK_FILENAME)
        left_lane = self.read_waypoints_array_from_csv(self.LEFT_TRACK_FILENAME)

        right_lane = self.inflate_track_boundaries(center_lane, right_lane, self.CAR_WIDTH, self.INFLATION_FACTOR)
        left_lane = self.inflate_track_boundaries(center_lane, left_lane, self.CAR_WIDTH, self.INFLATION_FACTOR)

        self.center_lane = np.row_stack((center_lane, center_lane[1:int(center_lane.shape[0] / 2), :]))
        self.right_lane = np.row_stack((right_lane, right_lane[1:int(center_lane.shape[0] / 2), :]))
        self.left_lane = np.row_stack((left_lane, left_lane[1:int(center_lane.shape[0] / 2), :]))
        center_derivative_data = np.row_stack(
            (center_derivative_data, center_derivative_data[1:int(center_lane.shape[0] / 2), :]))

        # Interpolate center line upto desired resolution
        self.element_arc_lengths_orig = self.get_arc_lengths(center_lane)
        self.element_arc_lengths = self.get_arc_lengths(self.center_lane)
        self.center_lut_x, self.center_lut_y = self.get_interpolated_path_casadi('lut_center_x', 'lut_center_y',
                                                                                 self.center_lane,
                                                                                 self.element_arc_lengths)
        self.center_lut_dx, self.center_lut_dy = self.get_interpolated_path_casadi('lut_center_dx', 'lut_center_dy',
                                                                                   center_derivative_data,
                                                                                   self.element_arc_lengths)
        self.center_point_angles = np.arctan2(center_derivative_data[:, 1], center_derivative_data[:, 0])

        # Interpolate right and left wall line
        self.right_lut_x, self.right_lut_y = self.get_interpolated_path_casadi('lut_right_x', 'lut_right_y', self.right_lane,
                                                                               self.element_arc_lengths)
        self.left_lut_x, self.left_lut_y = self.get_interpolated_path_casadi('lut_left_x', 'lut_left_y', self.left_lane,
                                                                             self.element_arc_lengths)

    def find_current_arc_length(self, car_pos):
        nearest_index, minimum_dist = self.find_nearest_index(car_pos)
        if minimum_dist > self.ARC_LENGTH_MIN_DIST_TOL:
            if nearest_index == 0:
                next_idx = 1
                prev_idx = self.center_lane.shape[0] - 1
            elif nearest_index == (self.center_lane.shape[0] - 1):
                next_idx = 0
                prev_idx = self.center_lane.shape[0] - 2
            else:
                next_idx = nearest_index + 1
                prev_idx = nearest_index - 1
            dot_product_value = np.dot(car_pos - self.center_lane[nearest_index, :],
                                       self.center_lane[prev_idx, :] - self.center_lane[nearest_index, :])
            if dot_product_value > 0:
                nearest_index_actual = prev_idx
            else:
                nearest_index_actual = nearest_index
                nearest_index = next_idx
            new_dot_value = np.dot(car_pos - self.center_lane[nearest_index_actual, :],
                                   self.center_lane[nearest_index, :] - self.center_lane[nearest_index_actual, :])
            projection = new_dot_value / np.linalg.norm(
                self.center_lane[nearest_index, :] - self.center_lane[nearest_index_actual, :])
            current_s = self.element_arc_lengths[nearest_index_actual] + projection
        else:
            current_s = self.element_arc_lengths[nearest_index]

        if nearest_index == 0:
            current_s = 0.0
        return current_s, nearest_index

    def controlLoopCB(self):
        '''Control loop for car MPC'''
        if self.goal_received and not self.goal_reached:
            control_loop_start_time = time.time()
            # Update system states: X=[x, y, psi]
            px = self.current_pos_x
            py = self.current_pos_y
            car_pos = np.array([self.current_pos_x, self.current_pos_y])
            psi = self.current_yaw

            # Update system inputs: U=[speed(v), steering]
            v = self.current_vel_odom
            steering = self.steering_angle  # radian
            L = self.mpc.L

            current_s, near_idx = self.find_current_arc_length(car_pos)

            # Lap counter: detect when s wraps around from max to near 0
            if self.racing_started and self.prev_s > self.param['s_max'] * 0.9 and current_s < self.param['s_max'] * 0.1:
                self.lap_count += 1
                self.get_logger().info(f"Lap {self.lap_count}/{self.max_laps} completed!")
                if self.lap_count >= self.max_laps:
                    self.get_logger().info(f"Race finished! {self.max_laps} laps completed!")
                    self.racing_started = False
                    self.goal_received = False
                    self.plot_data()

            self.prev_s = current_s
            self.get_logger().info(f"pre {current_s} {near_idx}", throttle_duration_sec=1.0)

            if self.DELAY_MODE:
                dt_lag = self.LAG_TIME
                px = px + v * np.cos(psi) * dt_lag
                py = py + v * np.sin(psi) * dt_lag
                psi = psi + (v / L) * tan(steering) * dt_lag
                current_s = current_s + self.projected_vel * dt_lag

            current_state = np.array([px, py, psi, current_s])

            centerPose = PoseStamped()
            centerPose.header = self.create_header('map')
            centerPose.pose.position.x = float(self.center_lane[near_idx, 0])
            centerPose.pose.position.y = float(self.center_lane[near_idx, 1])
            centerPose.pose.orientation = self.heading(self.center_point_angles[near_idx])
            self.center_tangent_pub.publish(centerPose)

            # Solve MPC Problem
            mpc_time = time.time()
            first_control, trajectory, control_inputs = self.mpc.solve(current_state)
            mpc_compute_time = time.time() - mpc_time

            # MPC result (all described in car frame)
            speed = float(first_control[0])  # speed
            steering = float(first_control[1])  # radian
            self.projected_vel = speed

            # throttle calculation
            throttle = 0.03 * (speed - v) / self.param['dT']

            if throttle > 1:
                throttle = 1
            elif throttle < -1:
                throttle = -1
            if speed == 0:
                throttle = 0

            if not self.mpc.WARM_START:
                speed, steering, throttle = 0, 0, 0
                self.mpc.WARM_START = True
            if (speed >= self.param['v_max']):
                speed = self.param['v_max']
            elif (speed <= (- self.param['v_max'] / 2.0)):
                speed = - self.param['v_max'] / 2.0

            # Display the MPC predicted trajectory
            mpc_traj = Path()
            mpc_traj.header = self.create_header('map')
            mpc_traj.poses = []
            for i in range(trajectory.shape[0]):
                tempPose = PoseStamped()
                tempPose.header = mpc_traj.header
                tempPose.pose.position.x = trajectory[i, 0]
                tempPose.pose.position.y = trajectory[i, 1]
                tempPose.pose.orientation = self.heading(trajectory[i, 2])
                mpc_traj.poses.append(tempPose)
            self.mpc_trajectory_pub.publish(mpc_traj)

            # TODO: Publish MPC metadata and prediction (requires custom messages)

            total_time = time.time() - control_loop_start_time
            if self.DEBUG_MODE:
                self.get_logger().info("DEBUG", throttle_duration_sec=1.0)
                self.get_logger().info(f"psi: {psi}", throttle_duration_sec=1.0)
                self.get_logger().info(f"V: {v}", throttle_duration_sec=1.0)
                self.get_logger().info(f"MPC Speed cmd: {speed}", throttle_duration_sec=1.0)
                self.get_logger().info(f"Steering cmd: {steering}", throttle_duration_sec=1.0)
                self.get_logger().info(f"Throttle: {throttle}", throttle_duration_sec=1.0)
                self.get_logger().info(f"Control loop time mpc= {mpc_compute_time}", throttle_duration_sec=1.0)
                self.get_logger().info(f"Control loop time= {total_time}", throttle_duration_sec=1.0)

            self.current_time += 1.0 / self.CONTROLLER_FREQ
            self.t_plot.append(self.current_time)
            self.v_plot.append(speed)
            self.steering_plot.append(np.rad2deg(steering))
            self.time_plot.append(mpc_compute_time * 1000)
        else:
            steering = 0.0
            speed = 0.0
            throttle = 0.0

        # publish cmd
        ackermann_cmd = AckermannDriveStamped()
        ackermann_cmd.header = self.create_header(self.car_frame)
        ackermann_cmd.drive.steering_angle = float(steering)
        self.steering_angle = steering
        ackermann_cmd.drive.speed = float(speed)
        if self.THROTTLE_MODE:
            ackermann_cmd.drive.acceleration = float(throttle)
        self.ackermann_pub.publish(ackermann_cmd)

    def plot_data(self):
        plt.figure(1)
        plt.subplot(411)
        plt.step(self.t_plot, self.v_plot, 'k', linewidth=1.5)
        plt.ylabel('v m/s')
        plt.xlabel('time(s)')
        plt.subplot(412)
        plt.step(self.t_plot, self.steering_plot, 'r', linewidth=1.5)
        plt.ylabel('steering angle(degrees)')
        plt.xlabel('time(s)')
        plt.subplot(414)
        plt.step(self.t_plot, self.time_plot, 'b', linewidth=1.5)
        plt.ylim(0.0, 100)
        plt.ylabel('mpc_compute_time in ms')
        plt.xlabel('time(s)')
        plt.show()

        self.t_plot = []
        self.steering_plot = []
        self.v_plot = []
        self.cte_plot = []
        self.time_plot = []
        self.current_time = 0


def main(args=None):
    rclpy.init(args=args)
    mpc_node = MPCKinematicNode()
    try:
        rclpy.spin(mpc_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print solver statistics before shutdown
        if hasattr(mpc_node, 'mpc') and mpc_node.mpc is not None:
            mpc_node.mpc.print_solver_statistics()
        mpc_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
