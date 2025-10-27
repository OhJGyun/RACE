#!/usr/bin/env python3
"""
Steering Lookup Table - Ported from race_stack to ROS2
Original: https://github.com/ForzaETH/race_stack
Changes: rospkg -> ament_index_python for ROS2
"""

import numpy as np
from ament_index_python.packages import get_package_share_directory
import os


def find_nearest(array, value):
    # array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx], idx


def find_closest_neighbors(array, value):
    # index of first nan
    is_nan_array = np.argwhere(np.isnan(array))
    if len(is_nan_array) > 0:
      first_nan = is_nan_array[0][0]
      array = array[0:first_nan]
    closest, closest_idx = find_nearest(array, value)
    if closest_idx == 0:
        return array[0], 0, array[0], 0
    elif closest_idx == (len(array) - 1):
        return array[closest_idx], closest_idx, array[closest_idx], closest_idx
    else:
      second_closest, second_idx = \
        find_nearest(array[[max(closest_idx-1, 0),
        min(closest_idx+1, len(array)-1)]], value)
      second_idx = -1 + closest_idx + 2 * second_idx
      return closest, closest_idx, second_closest, second_idx


class LookupSteerAngle:
    """
    LookupSteerAngle:
    """
    def __init__(self, model_name, logger):
      # ROS2: use ament_index_python instead of rospkg
      try:
          pkg_path = get_package_share_directory('map_controller')
          file_path = os.path.join(pkg_path, 'config', 'steering_lut', model_name + '_lookup_table.csv')
      except Exception as e:
          logger(f"[Steering Lookup] Package path error: {e}")
          # Fallback: try to use kinematic model
          file_path = None

      if file_path and os.path.exists(file_path):
          try:
              self.lu = np.loadtxt(file_path, delimiter=",")
              self.use_lut = True
              logger(f"[Steering Lookup] Loaded lookup table from {file_path}")
          except IOError:
              logger(f"[Steering Lookup] WARNING: Lookup table not found at {file_path}. Using kinematic fallback.")
              self.use_lut = False
      else:
          logger(f"[Steering Lookup] No lookup table specified or file not found. Using kinematic fallback.")
          self.use_lut = False

      self.logger = logger
      self.wheelbase = 0.33  # Default F1TENTH wheelbase

    def lookup_steer_angle(self, accel, vel):
        """
        lookup_steer_angle:
        """
        if not self.use_lut:
            # Kinematic fallback: simple Ackermann steering
            # steer = atan2(wheelbase * accel, vel^2)
            if vel < 0.1:
                return 0.0
            steer_angle = np.arctan2(self.wheelbase * accel, vel**2)
            return np.clip(steer_angle, -0.4, 0.4)

        # Original LUT logic
        if accel > 0.0:
          sign_accel = 1.0
        else:
          sign_accel = -1.0
        # lookup only for positive accelerations
        accel = abs(accel)
        lu_vs = self.lu[0, 1:]
        lu_steers = self.lu[1:, 0]

        #if (vel > lu_vs[-1]):
        # self.logger(5, "Velocity exceeds lookup table, generating steering angle for v :" + str(lu_vs[-1]))

        # find closest velocities to vel
        c_v, c_v_idx = find_nearest(lu_vs, vel)

        # find two closest accelerations to accel
        c_a, c_a_idx, s_a, s_a_idx = find_closest_neighbors(self.lu[1:, c_v_idx + 1], accel)
        if c_a_idx == s_a_idx:
          steer_angle = lu_steers[c_a_idx]
        else :
          # interpolate between two closest accelerations to find steering angle
          steer_angle = np.interp(accel, [c_a, s_a], [lu_steers[c_a_idx], lu_steers[s_a_idx]])
        return steer_angle * sign_accel
