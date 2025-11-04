#!/usr/bin/env python3
"""
MAP Controller Manager - Single Reference Frame Version (OJG)
Modified for multi-lane with unified s-coordinate system

Key changes from original:
- Single FrenetConverter based on optimal lane (lane 0)
- All lanes share the same s-coordinate system
- Supports full CSV format: x, y, v, d, s, kappa, psi, ax
- No FrenetConverter reinitialization during lane changes
- Simplified and optimized for consistent multi-lane control
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.time import Time as RclTime
from rclpy.duration import Duration
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, Int32, Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from transforms3d.euler import quat2euler
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException, Buffer, TransformListener
import csv

from .map_controller import MAP_Controller
from .frenet_converter import FrenetConverter


class ControllerManagerOJG(Node):
    """
    MAP Controller Manager with Single Reference Frame for Multi-Lane

    All lanes (optimal, left, right) share the same s-coordinate system from optimal lane.
    Only one FrenetConverter is used throughout the session.
    """

    def __init__(self):
        super().__init__('map_controller_ojg')

        # Declare parameters
        self.declare_ros_parameters()

        # Load parameters
        self.load_parameters()

        # Initialize TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State variables
        self.position_in_map = np.array([[0.0, 0.0, 0.0]])  # [x, y, theta]
        self.position_in_map_frenet = np.array([0.0, 0.0, 0.0, 0.0])  # [s, d, vs, vd]
        self.speed_now = 0.0
        self.acc_now = np.zeros(5)  # last 5 acceleration values
        self.max_speed_achieved = 0.0
        self.waypoint_array_in_map = None
        self.track_length = 0.0

        # 🎯 Single reference frame multi-lane support
        self.lane_waypoints = []  # List of all lane waypoint arrays
        self.current_lane_idx = 0  # Currently active lane index
        self.lane_change_active = False
        self.lane_change_start_time = None
        self.lane_change_duration = 1.0  # Duration to apply reduced LD (seconds)

        # 🔑 KEY: Only ONE FrenetConverter based on optimal lane (lane 0)
        self.frenet_converter = None  # Will be initialized with optimal lane only

        # Disparity mode (optional obstacle avoidance)
        self.disparity_mode_active = False
        self.disparity_start_time = None

        # Circular track optimization
        self.last_nearest_idx = None

        # Control state
        self.state = "RACING"
        self.opponent = None

        # Flags
        self.has_pose = False
        self.has_odom = False
        self.has_waypoints = False

        # For AMCL fallback
        self.current_pose = None
        self.current_pose_yaw = 0.0
        self.last_pose_time = None

        # Load waypoints
        self.load_waypoints()

        # Initialize MAP controller
        if self.has_waypoints:
            self.map_controller = MAP_Controller(
                waypoints=self.waypoint_array_in_map,
                t_clip_min=self.t_clip_min,
                t_clip_max=self.t_clip_max,
                m_l1=self.m_l1,
                q_l1=self.q_l1,
                speed_lookahead=self.speed_lookahead,
                speed_lookahead_for_steer=self.speed_lookahead_for_steer,
                lat_err_coeff=self.lat_err_coeff,
                acc_scaler_for_steer=self.acc_scaler_for_steer,
                dec_scaler_for_steer=self.dec_scaler_for_steer,
                start_scale_speed=self.start_scale_speed,
                end_scale_speed=self.end_scale_speed,
                downscale_factor=self.downscale_factor,
                steering_lut=self.steering_lut
            )
        else:
            self.map_controller = None

        # Setup ROS publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 1)
        self.lookahead_pub = self.create_publisher(PoseStamped, '/map_controller/lookahead_point', 1)
        self.path_pub = self.create_publisher(Path, '/map_controller/path', 1)
        self.drive_mode_pub = self.create_publisher(Int32, '/drive_mode', 1)

        # Setup ROS subscribers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_topic, self.amcl_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile)
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_profile)

        # Lane selector subscriber
        self.lane_selector_sub = self.create_subscription(
            Int32, self.lane_selector_topic, self.lane_selector_callback, 10)

        # Lane segment subscriber (optional)
        self.lane_segment_sub = self.create_subscription(
            Int32MultiArray, self.lane_segment_topic, self.lane_segment_callback, 10)

        # Control loop timer
        self.timer = self.create_timer(1.0 / self.loop_rate_hz, self.control_loop)

        self.get_logger().info("=" * 60)
        self.get_logger().info("🏎️  MAP Controller OJG (Single Reference Frame)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"✅ Loaded {len(self.lane_waypoints)} lanes")
        if self.lane_waypoints:
            for i, lane in enumerate(self.lane_waypoints):
                self.get_logger().info(f"   Lane {i}: {len(lane)} waypoints")
        self.get_logger().info(f"🎯 Using SINGLE FrenetConverter (optimal lane reference)")
        self.get_logger().info(f"📍 Localization: TF={self.use_tf_for_localization}, "
                              f"AMCL={self.use_amcl_pose}, Odom={self.use_odom_pose}")
        self.get_logger().info(f"🏁 Control loop: {self.loop_rate_hz} Hz")
        self.get_logger().info("=" * 60)

    def declare_ros_parameters(self):
        """Declare all ROS parameters with defaults"""
        # Waypoint configuration
        self.declare_parameter('csv_file_path', '/home/ircv7/RACE/path/lane_optimal.csv')
        self.declare_parameter('lane_csv_paths', [])
        self.declare_parameter('lane_selector_topic', '/lane_selector/target_lane')
        self.declare_parameter('lane_segment_topic', '/lane_selector/current_segment')

        # Lane change behavior
        self.declare_parameter('lane_change_ld_gain', 0.7)
        self.declare_parameter('lane_change_speed_gain', 0.7)
        self.declare_parameter('scc_speed_gain', 0.7)

        # Disparity avoidance (optional)
        self.declare_parameter('use_disparity', False)
        self.declare_parameter('disparity_duration', 1.0)
        self.declare_parameter('disparity_max_range', 10.0)
        self.declare_parameter('disparity_threshold', 1.0)
        self.declare_parameter('disparity_car_half_width', 0.15)
        self.declare_parameter('disparity_margin', 0.3)
        self.declare_parameter('disparity_max_speed', 4.5)
        self.declare_parameter('disparity_min_speed', 4.0)
        self.declare_parameter('disparity_max_steering', 0.4)
        self.declare_parameter('disparity_ld_min', 0.5)
        self.declare_parameter('disparity_ld_max', 5.0)
        self.declare_parameter('disparity_m_ld', 0.4)
        self.declare_parameter('disparity_q_ld', 0.3)
        self.declare_parameter('disparity_steering_alpha', 0.7)

        # Localization configuration
        self.declare_parameter('use_tf_for_localization', True)
        self.declare_parameter('tf_timeout', 0.1)
        self.declare_parameter('max_pose_age', 0.5)
        self.declare_parameter('use_amcl_pose', False)
        self.declare_parameter('use_odom_pose', False)

        # Topic configuration
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('drive_topic', '/drive')

        # L1 controller parameters
        self.declare_parameter('t_clip_min', 2.5)
        self.declare_parameter('t_clip_max', 5.0)
        self.declare_parameter('m_l1', 0.6)
        self.declare_parameter('q_l1', 0.1)
        self.declare_parameter('speed_lookahead', 0.25)
        self.declare_parameter('speed_lookahead_for_steer', 0.1)
        self.declare_parameter('lat_err_coeff', 0.5)
        self.declare_parameter('acc_scaler_for_steer', 1.2)
        self.declare_parameter('dec_scaler_for_steer', 1.2)
        self.declare_parameter('start_scale_speed', 7.0)
        self.declare_parameter('end_scale_speed', 8.0)
        self.declare_parameter('downscale_factor', 0.0)

        # Steering lookup table
        self.declare_parameter('steering_lut', 'NUC2_pacejka')

        # Control loop
        self.declare_parameter('loop_rate_hz', 40.0)

    def load_parameters(self):
        """Load all parameters from ROS parameter server"""
        # Waypoint configuration
        self.csv_file_path = self.get_parameter('csv_file_path').value
        self.lane_csv_paths = self.get_parameter('lane_csv_paths').value
        self.lane_selector_topic = self.get_parameter('lane_selector_topic').value
        self.lane_segment_topic = self.get_parameter('lane_segment_topic').value

        # Lane change behavior
        self.lane_change_ld_gain = self.get_parameter('lane_change_ld_gain').value
        self.lane_change_speed_gain = self.get_parameter('lane_change_speed_gain').value
        self.scc_speed_gain = self.get_parameter('scc_speed_gain').value

        # Lateral acceleration limit
        self.lat_accel_max = self.get_parameter('lat_accel_max').value

        # Disparity avoidance
        self.use_disparity = self.get_parameter('use_disparity').value
        self.disparity_duration = self.get_parameter('disparity_duration').value
        self.disparity_max_range = self.get_parameter('disparity_max_range').value
        self.disparity_threshold = self.get_parameter('disparity_threshold').value
        self.disparity_car_half_width = self.get_parameter('disparity_car_half_width').value
        self.disparity_margin = self.get_parameter('disparity_margin').value
        self.disparity_max_speed = self.get_parameter('disparity_max_speed').value
        self.disparity_min_speed = self.get_parameter('disparity_min_speed').value
        self.disparity_max_steering = self.get_parameter('disparity_max_steering').value
        self.disparity_ld_min = self.get_parameter('disparity_ld_min').value
        self.disparity_ld_max = self.get_parameter('disparity_ld_max').value
        self.disparity_m_ld = self.get_parameter('disparity_m_ld').value
        self.disparity_q_ld = self.get_parameter('disparity_q_ld').value
        self.disparity_steering_alpha = self.get_parameter('disparity_steering_alpha').value

        # Localization configuration
        self.use_tf_for_localization = self.get_parameter('use_tf_for_localization').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        self.max_pose_age = self.get_parameter('max_pose_age').value
        self.use_amcl_pose = self.get_parameter('use_amcl_pose').value
        self.use_odom_pose = self.get_parameter('use_odom_pose').value

        # Topic configuration
        self.odom_topic = self.get_parameter('odom_topic').value
        self.amcl_topic = self.get_parameter('amcl_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value

        # L1 controller parameters
        self.t_clip_min = self.get_parameter('t_clip_min').value
        self.t_clip_max = self.get_parameter('t_clip_max').value
        self.m_l1 = self.get_parameter('m_l1').value
        self.q_l1 = self.get_parameter('q_l1').value
        self.speed_lookahead = self.get_parameter('speed_lookahead').value
        self.speed_lookahead_for_steer = self.get_parameter('speed_lookahead_for_steer').value
        self.lat_err_coeff = self.get_parameter('lat_err_coeff').value
        self.acc_scaler_for_steer = self.get_parameter('acc_scaler_for_steer').value
        self.dec_scaler_for_steer = self.get_parameter('dec_scaler_for_steer').value
        self.start_scale_speed = self.get_parameter('start_scale_speed').value
        self.end_scale_speed = self.get_parameter('end_scale_speed').value
        self.downscale_factor = self.get_parameter('downscale_factor').value

        # Steering lookup table
        self.steering_lut = self.get_parameter('steering_lut').value

        # Control loop
        self.loop_rate_hz = self.get_parameter('loop_rate_hz').value

    def load_waypoints(self):
        """
        🎯 Load waypoints with SINGLE reference frame approach

        All lanes must have:
        - Same number of waypoints
        - Same s coordinates (from optimal lane)
        - Different d offsets (lane position)
        - Individual x, y, v profiles
        """
        # Load all lanes
        if self.lane_csv_paths and len(self.lane_csv_paths) > 0:
            self.get_logger().info(f"Loading {len(self.lane_csv_paths)} lane files...")

            for i, path in enumerate(self.lane_csv_paths):
                waypoints = self._load_full_waypoint_file(path)
                if waypoints is not None:
                    self.lane_waypoints.append(waypoints)
                    self.get_logger().info(
                        f"  Lane {i}: {len(waypoints)} points, "
                        f"track_length={waypoints[-1, 4]:.2f}m, "
                        f"d_offset={waypoints[0, 3]:.3f}m"
                    )
                else:
                    self.get_logger().error(f"Failed to load lane {i}: {path}")
                    return
        else:
            # Fallback: load single file
            waypoints = self._load_full_waypoint_file(self.csv_file_path)
            if waypoints is not None:
                self.lane_waypoints.append(waypoints)
                self.get_logger().info(f"Loaded single lane: {len(waypoints)} waypoints")
            else:
                self.get_logger().error(f"Failed to load waypoints from {self.csv_file_path}")
                return

        # Validate: all lanes must have same number of waypoints
        if len(self.lane_waypoints) > 1:
            ref_count = len(self.lane_waypoints[0])
            for i, lane in enumerate(self.lane_waypoints[1:], 1):
                if len(lane) != ref_count:
                    self.get_logger().error(
                        f"❌ Lane {i} has {len(lane)} waypoints, "
                        f"but lane 0 has {ref_count} waypoints! "
                        f"All lanes must have the SAME waypoint count for single reference frame."
                    )
                    self.has_waypoints = False
                    return

        # Set current lane to optimal (lane 0)
        self.current_lane_idx = 0
        self.waypoint_array_in_map = self.lane_waypoints[0]
        self.track_length = self.waypoint_array_in_map[-1, 4]
        self.has_waypoints = True

        # 🔑 KEY: Initialize SINGLE FrenetConverter with optimal lane (lane 0)
        try:
            optimal_lane = self.lane_waypoints[0]
            self.frenet_converter = FrenetConverter(
                waypoints_x=optimal_lane[:, 0],
                waypoints_y=optimal_lane[:, 1],
                waypoints_psi=optimal_lane[:, 6]
            )
            self.get_logger().info("✅ Initialized FrenetConverter with optimal lane (will be used for ALL lanes)")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize FrenetConverter: {e}")
            self.has_waypoints = False

    def _load_full_waypoint_file(self, csv_path: str):
        """
        Load waypoints from CSV file with full format
        Expected format: x, y, v, d, s, kappa, psi, ax

        Returns numpy array [N x 8] or None if failed
        """
        waypoints = []
        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) < 2:
                        continue

                    # Skip header/comment lines
                    if row[0].strip().startswith('#') or not row[0].strip():
                        continue

                    try:
                        x = float(row[0])
                        y = float(row[1])
                        v = float(row[2]) if len(row) > 2 else 2.0
                        d = float(row[3]) if len(row) > 3 else 0.0
                        s = float(row[4]) if len(row) > 4 else 0.0
                        kappa = float(row[5]) if len(row) > 5 else 0.0
                        psi = float(row[6]) if len(row) > 6 else 0.0
                        ax = float(row[7]) if len(row) > 7 else 0.0
                    except ValueError:
                        continue

                    # Check for duplicates
                    if waypoints and abs(waypoints[-1][0] - x) < 1e-4 and abs(waypoints[-1][1] - y) < 1e-4:
                        continue

                    waypoints.append([x, y, v, d, s, kappa, psi, ax])

            if len(waypoints) < 2:
                self.get_logger().error(f"Not enough waypoints in {csv_path}")
                return None

            return np.array(waypoints)

        except Exception as e:
            self.get_logger().error(f"Failed to load {csv_path}: {e}")
            return None

    def get_current_pose(self):
        """Get current pose from TF, fallback to AMCL/odom"""
        if self.use_tf_for_localization:
            try:
                # Lookup transform from map to base_link
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_timeout)
                )

                # Extract position
                x = transform.transform.translation.x
                y = transform.transform.translation.y

                # Extract orientation (yaw)
                quat = transform.transform.rotation
                _, _, theta = quat2euler([quat.w, quat.x, quat.y, quat.z])

                # Update pose
                self._update_pose(x, y, theta)
                return np.array([x, y, theta])

            except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as e:
                # Fallback to AMCL if TF fails
                if self.use_amcl_pose and self.current_pose is not None:
                    # Check if pose is recent
                    now = self.get_clock().now()
                    pose_age = (now - RclTime.from_msg(self.last_pose_time)).nanoseconds * 1e-9

                    if pose_age < self.max_pose_age:
                        x = self.current_pose.pose.pose.position.x
                        y = self.current_pose.pose.pose.position.y
                        self._update_pose(x, y, self.current_pose_yaw)
                        return np.array([x, y, self.current_pose_yaw])

                return None

        # If not using TF, pose should already be updated by callbacks
        if self.has_pose:
            return self.position_in_map[0]

        return None

    def _update_pose(self, x: float, y: float, theta: float):
        """
        🎯 Update stored pose and Frenet coordinates

        Uses SINGLE FrenetConverter (optimal lane) for all lanes
        """
        self.position_in_map = np.array([[x, y, theta]])

        # Frenet conversion using optimal lane reference
        if self.has_waypoints and self.frenet_converter is not None:
            try:
                # Get Frenet coordinates (s, d) relative to optimal lane
                s, d = self.frenet_converter.get_frenet(np.array([x]), np.array([y]))

                # Get Frenet velocities (vs, vd)
                if self.has_odom:
                    vx = self.speed_now * np.cos(theta)
                    vy = self.speed_now * np.sin(theta)
                    vs, vd = self.frenet_converter.get_frenet_velocities(vx, vy, theta, s[0])
                    self.position_in_map_frenet = np.array([s[0], d[0], vs, vd])
                else:
                    self.position_in_map_frenet = np.array([s[0], d[0], 0.0, 0.0])
            except Exception as e:
                self.get_logger().warn(f"Frenet conversion failed: {e}", throttle_duration_sec=5.0)

        self.has_pose = True

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """AMCL pose callback"""
        self.current_pose = msg
        orientation = msg.pose.pose.orientation
        _, _, self.current_pose_yaw = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])
        self.last_pose_time = msg.header.stamp

        if not self.use_tf_for_localization:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self._update_pose(x, y, self.current_pose_yaw)
        else:
            self.has_pose = True

    def odom_callback(self, msg: Odometry):
        """Odometry callback"""
        self.speed_now = msg.twist.twist.linear.x

        if self.use_odom_pose or not self.has_pose:
            pose = msg.pose.pose
            x = pose.position.x
            y = pose.position.y
            _, _, theta = quat2euler([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
            self._update_pose(x, y, theta)

        # Update Frenet velocities
        if self.has_pose and self.has_waypoints and self.frenet_converter is not None:
            try:
                theta = self.position_in_map[0, 2]
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y
                s = self.position_in_map_frenet[0]
                vs, vd = self.frenet_converter.get_frenet_velocities(vx, vy, theta, s)
                self.position_in_map_frenet[2] = vs
                self.position_in_map_frenet[3] = vd
            except Exception as e:
                pass

        self.has_odom = True

    def imu_callback(self, msg: Imu):
        """IMU callback"""
        self.acc_now[1:] = self.acc_now[:-1]
        self.acc_now[0] = msg.linear_acceleration.x

    def lane_selector_callback(self, msg: Int32):
        """
        🎯 Lane selector callback - SIMPLIFIED for single reference frame

        No FrenetConverter reinitialization needed!
        """
        desired_lane = msg.data

        # Validate lane index
        if desired_lane < 0 or desired_lane >= len(self.lane_waypoints):
            self.get_logger().warn(f"Invalid lane index {desired_lane}, ignoring")
            return

        # Check if lane actually changed
        if desired_lane == self.current_lane_idx:
            return

        # Switch lane
        prev_lane = self.current_lane_idx
        self.current_lane_idx = desired_lane
        self.waypoint_array_in_map = self.lane_waypoints[desired_lane]

        # ✅ track_length is ALWAYS the same (shared s-coordinate system)
        # No need to update track_length or FrenetConverter!

        # Reset nearest waypoint cache for new lane geometry
        self.last_nearest_idx = None

        # Activate lane change mode
        self.lane_change_active = True
        self.lane_change_start_time = self.get_clock().now().nanoseconds * 1e-9

        # Determine lane change type
        is_avoidance = (prev_lane == 0 and desired_lane != 0)
        is_return = (prev_lane != 0 and desired_lane == 0)

        # Publish driving mode
        drive_mode_msg = Int32()
        drive_mode_msg.data = 1  # AVOIDANCE mode
        self.drive_mode_pub.publish(drive_mode_msg)

        # Optional: activate disparity mode for obstacle avoidance
        if self.use_disparity and is_avoidance:
            self.disparity_mode_active = True
            self.disparity_start_time = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
                f"🛣️ Lane AVOIDANCE: {prev_lane} → {desired_lane}, 🚨 DISPARITY MODE for {self.disparity_duration:.1f}s"
            )
        elif is_return:
            self.get_logger().info(
                f"🛣️ Lane RETURN: {prev_lane} → {desired_lane}, using map control with reduced LD/speed"
            )
        else:
            self.get_logger().info(
                f"🛣️ Lane switched: {prev_lane} → {desired_lane} ({len(self.waypoint_array_in_map)} waypoints), "
                f"LD×{self.lane_change_ld_gain:.2f}, Speed×{self.lane_change_speed_gain:.2f}"
            )

    def lane_segment_callback(self, msg: Int32MultiArray):
        """Lane segment callback (optional, for local waypoint window)"""
        # Not implemented in simplified version
        pass

    def nearest_waypoint(self, position, window_size=50):
        """Find index of nearest waypoint to position"""
        if not self.has_waypoints:
            return 0

        waypoints = self.waypoint_array_in_map[:, :2]
        num_points = len(waypoints)

        # First call or after lane change: full search
        if self.last_nearest_idx is None or self.last_nearest_idx >= num_points:
            position_array = np.array([position] * num_points)
            distances = np.linalg.norm(position_array - waypoints, axis=1)
            self.last_nearest_idx = int(np.argmin(distances))
            return self.last_nearest_idx

        # Local window search for performance
        if num_points <= window_size * 2:
            position_array = np.array([position] * num_points)
            distances = np.linalg.norm(position_array - waypoints, axis=1)
            self.last_nearest_idx = int(np.argmin(distances))
            return self.last_nearest_idx

        start_idx = max(0, self.last_nearest_idx - window_size)
        end_idx = min(num_points, self.last_nearest_idx + window_size)

        local_waypoints = waypoints[start_idx:end_idx]
        position_array = np.array([position] * len(local_waypoints))
        distances = np.linalg.norm(position_array - local_waypoints, axis=1)
        local_min_idx = int(np.argmin(distances))

        self.last_nearest_idx = start_idx + local_min_idx
        return self.last_nearest_idx

    def control_loop(self):
        """Main control loop"""
        # Wait for initialization
        if not self.has_odom or not self.has_waypoints:
            self.get_logger().warn(
                f"Waiting: odom={self.has_odom}, waypoints={self.has_waypoints}",
                throttle_duration_sec=2.0
            )
            return

        # Get current pose
        pose = self.get_current_pose()
        if pose is None:
            self.get_logger().warn(
                "No valid pose available (TF/AMCL/odom all failed)",
                throttle_duration_sec=2.0
            )
            return

        # Check if lane change duration has elapsed
        if self.lane_change_active:
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.lane_change_start_time
            if elapsed > self.lane_change_duration:
                self.lane_change_active = False
                self.get_logger().info("✅ Lane change completed, restoring normal LD/speed")

                # Publish RACING mode
                drive_mode_msg = Int32()
                drive_mode_msg.data = 0  # RACING mode
                self.drive_mode_pub.publish(drive_mode_msg)

        # Check if disparity mode duration has elapsed
        if self.disparity_mode_active:
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.disparity_start_time
            if elapsed > self.disparity_duration:
                self.disparity_mode_active = False
                self.get_logger().info("✅ Disparity mode ended, resuming path following")

        # MAP controller
        if self.map_controller is not None:
            # Apply lane change gains if active
            # Note: These multipliers are applied internally in map_controller if needed
            # For now, we pass waypoints directly and controller handles everything

            # Call MAP controller main_loop (FORZA-compatible interface)
            speed, acceleration, jerk, steering, L1_point, L1_distance, idx_nearest = \
                self.map_controller.main_loop(
                    state=self.state,
                    position_in_map=self.position_in_map,
                    waypoint_array_in_map=self.waypoint_array_in_map,
                    speed_now=self.speed_now,
                    opponent=self.opponent,
                    position_in_map_frenet=self.position_in_map_frenet,
                    acc_now=self.acc_now,
                    track_length=self.track_length
                )

            # Apply lane change speed multiplier if active
            if self.lane_change_active:
                speed *= self.lane_change_speed_gain

            # Store L1 point for visualization
            self.map_controller.l1_point = L1_point

            # Publish drive command
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = 'base_link'
            drive_msg.drive.steering_angle = steering
            drive_msg.drive.speed = speed
            self.drive_pub.publish(drive_msg)

            # Track max speed
            if self.speed_now > self.max_speed_achieved:
                self.max_speed_achieved = self.speed_now

            # Visualization (throttled)
            if hasattr(self.map_controller, 'l1_point'):
                lookahead_msg = PoseStamped()
                lookahead_msg.header.stamp = self.get_clock().now().to_msg()
                lookahead_msg.header.frame_id = 'map'
                lookahead_msg.pose.position.x = self.map_controller.l1_point[0]
                lookahead_msg.pose.position.y = self.map_controller.l1_point[1]
                self.lookahead_pub.publish(lookahead_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = ControllerManagerOJG()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.get_logger().info(f"🏁 Max speed achieved: {controller.max_speed_achieved:.2f} m/s")
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
