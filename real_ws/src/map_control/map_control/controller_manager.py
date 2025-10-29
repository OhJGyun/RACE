#!/usr/bin/env python3
"""
MAP Controller Manager - Unified version combining map_controller and map_foxy
Original: https://github.com/ForzaETH/race_stack

Key features:
- Multi-lane support with dynamic lane switching (from map_controller)
- TF2-based real-time localization (from map_foxy)
- Fallback to AMCL topic and Odom (from map_foxy)
- Lane selector integration for obstacle avoidance (from map_controller)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int32
from transforms3d.euler import quat2euler
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException, Buffer, TransformListener
import csv

from .map_controller import MAP_Controller
from .frenet_converter import FrenetConverter


class ControllerManager(Node):
    """
    Unified MAP Controller Manager for F1TENTH

    Features:
    - TF2-based real-time localization (map -> base_link)
    - Multi-lane support with dynamic switching
    - Lane selector integration for obstacle avoidance
    - Fallback to AMCL/Odom when TF unavailable

    Subscribes to:
    - TF (map -> base_link): real-time ego car position
    - /amcl_pose: fallback ego car position (AMCL)
    - /odom: ego car velocity and fallback position
    - /imu/data: acceleration for steering scaling (optional)
    - /lane_selector/target_lane: lane index for dynamic switching

    Publishes to:
    - /drive: Ackermann drive commands
    - /map_controller/lookahead_point: L1 lookahead visualization
    - /map_controller/path: global waypoint path
    """

    def __init__(self):
        super().__init__('map_controller')

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
        self.waypoint_array_in_map = None
        self.track_length = 0.0
        self.lanes = []
        self.current_lane_idx = 0

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
        self.last_pose_time = self.get_clock().now().to_msg()

        # Load waypoints from CSV (multi-lane support)
        self.load_lane_waypoints()

        # Initialize MAP controller
        self.get_logger().info("Initializing MAP Controller...")
        self.map_controller = MAP_Controller(
            t_clip_min=self.t_clip_min,
            t_clip_max=self.t_clip_max,
            m_l1=self.m_l1,
            q_l1=self.q_l1,
            speed_lookahead=self.speed_lookahead,
            lat_err_coeff=self.lat_err_coeff,
            acc_scaler_for_steer=self.acc_scaler_for_steer,
            dec_scaler_for_steer=self.dec_scaler_for_steer,
            start_scale_speed=self.start_scale_speed,
            end_scale_speed=self.end_scale_speed,
            downscale_factor=self.downscale_factor,
            speed_lookahead_for_steer=self.speed_lookahead_for_steer,

            prioritize_dyn=False,
            trailing_gap=0.0,
            trailing_p_gain=0.0,
            trailing_i_gain=0.0,
            trailing_d_gain=0.0,
            blind_trailing_speed=0.0,

            loop_rate=self.loop_rate,
            LUT_name=self.LUT_name,
            state_machine_rate=self.loop_rate,

            logger_info=self.get_logger().info,
            logger_warn=self.get_logger().warn
        )

        # Publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.lookahead_pub = self.create_publisher(PoseStamped, '/map_controller/lookahead_point', 10)
        self.lookahead_distance_pub = self.create_publisher(Float32, '/map_controller/lookahead_distance', 10)
        self.path_pub = self.create_publisher(Path, '/map_controller/path', 10)
        self.waypoints_pose_pub = self.create_publisher(PoseArray, '/map_controller/waypoints_pose', 10)
        self._path_published_once = False

        # Subscribers
        qos_sensor = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # AMCL subscription (for fallback)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_topic,
            self.amcl_callback,
            10
        )

        if self.use_tf_for_localization:
            self.get_logger().info(f"Using TF (map->base_link) for localization, {self.amcl_topic} as fallback")
        elif self.use_amcl_pose:
            self.get_logger().info(f"Using {self.amcl_topic} topic for localization")
        elif self.use_odom_pose:
            self.get_logger().info(f"Using {self.odom_topic} for localization (SIM MODE)")

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile=qos_sensor
        )
        self.get_logger().info(f"Subscribed to {self.odom_topic} for velocity")

        # Optional IMU for acceleration
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile=qos_sensor
        )
        self.get_logger().info(f"Subscribed to {self.imu_topic} for acceleration (optional)")

        # Lane selector integration
        self.lane_selector_topic = self.get_parameter('lane_selector_topic').value
        self.lane_idx_sub = self.create_subscription(
            Int32,
            self.lane_selector_topic,
            self.lane_index_callback,
            10
        )
        self.get_logger().info(f"Subscribed to {self.lane_selector_topic} for lane changes")

        # Control loop timer
        self.timer = self.create_timer(1.0 / self.loop_rate, self.control_loop)

        # Publish global path periodically
        self.publish_global_path()
        self.path_timer = self.create_timer(1.0, self.publish_global_path)

        self.get_logger().info("MAP Controller (Unified) initialized! Waiting for pose and odometry...")

    @staticmethod
    def _yaw_to_quaternion(yaw: float):
        half = 0.5 * yaw
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def declare_ros_parameters(self):
        """Declare all ROS2 parameters"""
        # CSV file paths (multi-lane support)
        self.declare_parameter('csv_file_path', '')
        self.declare_parameter('lane_csv_paths', [''])

        # L1 controller parameters
        self.declare_parameter('t_clip_min', 0.8)
        self.declare_parameter('t_clip_max', 5.0)
        self.declare_parameter('m_l1', 0.6)
        self.declare_parameter('q_l1', -0.18)
        self.declare_parameter('speed_lookahead', 0.25)
        self.declare_parameter('lat_err_coeff', 1.0)
        self.declare_parameter('acc_scaler_for_steer', 1.2)
        self.declare_parameter('dec_scaler_for_steer', 0.9)
        self.declare_parameter('start_scale_speed', 7.0)
        self.declare_parameter('end_scale_speed', 8.0)
        self.declare_parameter('downscale_factor', 0.2)
        self.declare_parameter('speed_lookahead_for_steer', 0.0)

        # Steering lookup table
        self.declare_parameter('steering_lut', '')

        # Loop rate
        self.declare_parameter('loop_rate_hz', 40.0)

        # Localization settings (TF-first approach)
        self.declare_parameter('use_tf_for_localization', True)
        self.declare_parameter('tf_timeout', 0.1)
        self.declare_parameter('max_pose_age', 0.5)

        # Legacy fallback options
        self.declare_parameter('use_odom_pose', False)
        self.declare_parameter('use_amcl_pose', False)

        # Topic names
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('lane_selector_topic', '/lane_selector/target_lane')

    def load_parameters(self):
        """Load all parameters from ROS2 parameter server"""
        self.csv_file_path = self.get_parameter('csv_file_path').value
        lane_paths_param = self.get_parameter('lane_csv_paths').value
        if lane_paths_param is None:
            lane_paths_param = []
        self.lane_csv_paths = [p for p in lane_paths_param if isinstance(p, str) and p]

        # Merge csv_file_path into lane_csv_paths
        if self.csv_file_path:
            if not self.lane_csv_paths:
                self.lane_csv_paths.append(self.csv_file_path)
            elif self.csv_file_path not in self.lane_csv_paths:
                self.lane_csv_paths.insert(0, self.csv_file_path)

        # Remove duplicates
        unique_paths = []
        for path in self.lane_csv_paths:
            if path not in unique_paths:
                unique_paths.append(path)
        self.lane_csv_paths = unique_paths

        self.t_clip_min = self.get_parameter('t_clip_min').value
        self.t_clip_max = self.get_parameter('t_clip_max').value
        self.m_l1 = self.get_parameter('m_l1').value
        self.q_l1 = self.get_parameter('q_l1').value
        self.speed_lookahead = self.get_parameter('speed_lookahead').value
        self.lat_err_coeff = self.get_parameter('lat_err_coeff').value
        self.acc_scaler_for_steer = self.get_parameter('acc_scaler_for_steer').value
        self.dec_scaler_for_steer = self.get_parameter('dec_scaler_for_steer').value
        self.start_scale_speed = self.get_parameter('start_scale_speed').value
        self.end_scale_speed = self.get_parameter('end_scale_speed').value
        self.downscale_factor = self.get_parameter('downscale_factor').value
        self.speed_lookahead_for_steer = self.get_parameter('speed_lookahead_for_steer').value

        self.LUT_name = self.get_parameter('steering_lut').value
        self.loop_rate = self.get_parameter('loop_rate_hz').value

        # TF localization parameters
        self.use_tf_for_localization = bool(self.get_parameter('use_tf_for_localization').value)
        self.tf_timeout = self.get_parameter('tf_timeout').value
        self.max_pose_age = self.get_parameter('max_pose_age').value

        # Legacy fallback
        self.use_odom_pose = bool(self.get_parameter('use_odom_pose').value)
        self.use_amcl_pose = bool(self.get_parameter('use_amcl_pose').value)

        # Topic names
        self.odom_topic = self.get_parameter('odom_topic').value
        self.amcl_topic = self.get_parameter('amcl_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value

    def load_lane_waypoints(self):
        """Load waypoints for every configured lane CSV."""
        if not self.lane_csv_paths:
            self.get_logger().error("No lane CSV paths specified!")
            return

        loaded_lanes = []
        reference_profile = None
        for idx, path in enumerate(self.lane_csv_paths):
            try:
                lane = self._load_waypoints_for_path(path, idx)
                # Speed profile interpolation if missing
                if not lane.get('has_speed_column') and reference_profile:
                    ref_s = reference_profile['s']
                    ref_speed = reference_profile['speed']
                    lane_s = lane['waypoints'][:, 4]
                    lane['waypoints'][:, 2] = np.interp(
                        lane_s,
                        ref_s,
                        ref_speed,
                        left=ref_speed[0],
                        right=ref_speed[-1],
                    )
                    lane['has_speed_column'] = True
                    self.get_logger().warn(
                        f"Lane {idx} ({path}) missing speed column. "
                        "Interpolated speed profile from reference lane."
                    )
                elif not lane.get('has_speed_column'):
                    self.get_logger().warn(
                        f"Lane {idx} ({path}) missing speed column. Using fallback speed of 2.0 m/s."
                    )
                elif lane.get('has_speed_column') and reference_profile is None:
                    reference_profile = {
                        's': lane['waypoints'][:, 4].copy(),
                        'speed': lane['waypoints'][:, 2].copy(),
                    }
                loaded_lanes.append(lane)
            except Exception as exc:
                self.get_logger().error(f"Failed to load waypoints from {path}: {exc}")

        if not loaded_lanes:
            self.get_logger().error("Unable to load any lane waypoints. MAP controller inactive.")
            return

        self.lanes = loaded_lanes

        # Set default lane (prefer csv_file_path match)
        default_lane_idx = 0
        if self.csv_file_path:
            for idx, lane in enumerate(self.lanes):
                if self.csv_file_path in lane['csv_path']:
                    default_lane_idx = idx
                    self.get_logger().info(f"Default lane set to index {idx} (csv_file_path match)")
                    break

        self.apply_lane(min(default_lane_idx, len(self.lanes) - 1))

    def _load_waypoints_for_path(self, csv_path: str, lane_index: int):
        """Load and process a single lane CSV file."""
        waypoints = []
        missing_speed = False
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 2:
                    continue

                # Skip header rows
                if row[0].startswith('#') or not row[0][0].isdigit() and row[0][0] != '-':
                    continue

                try:
                    x = float(row[0])
                    y = float(row[1])
                except ValueError:
                    continue

                speed = 2.0
                if len(row) > 2:
                    try:
                        speed = float(row[2])
                    except ValueError:
                        missing_speed = True
                else:
                    missing_speed = True

                # Skip duplicate points
                if waypoints and abs(waypoints[-1][0] - x) < 1e-4 and abs(waypoints[-1][1] - y) < 1e-4:
                    continue

                d = 0.0
                s = 0.0
                kappa = 0.0
                psi = 0.0
                ax = 0.0

                waypoints.append([x, y, speed, d, s, kappa, psi, ax])

        if len(waypoints) < 2:
            raise RuntimeError(f"Not enough waypoints in {csv_path}")

        waypoints = np.array(waypoints)

        # Compute cumulative distance (s)
        for i in range(1, len(waypoints)):
            dx = waypoints[i, 0] - waypoints[i-1, 0]
            dy = waypoints[i, 1] - waypoints[i-1, 1]
            waypoints[i, 4] = waypoints[i-1, 4] + np.sqrt(dx**2 + dy**2)

        track_length = waypoints[-1, 4]

        # Remove duplicate closing point
        if len(waypoints) > 1 and abs(waypoints[0, 0] - waypoints[-1, 0]) < 1e-4 and abs(waypoints[0, 1] - waypoints[-1, 1]) < 1e-4:
            waypoints = waypoints[:-1]
            track_length = waypoints[-1, 4]

        # Compute heading (psi)
        for i in range(len(waypoints) - 1):
            dx = waypoints[i+1, 0] - waypoints[i, 0]
            dy = waypoints[i+1, 1] - waypoints[i, 1]
            waypoints[i, 6] = np.arctan2(dy, dx)
        waypoints[-1, 6] = waypoints[-2, 6]

        # Compute curvature (kappa)
        for i in range(1, len(waypoints) - 1):
            psi_prev = waypoints[i-1, 6]
            psi_next = waypoints[i+1, 6]
            ds = waypoints[i+1, 4] - waypoints[i-1, 4]
            if ds > 0:
                waypoints[i, 5] = (psi_next - psi_prev) / ds

        frenet_converter = FrenetConverter(
            waypoints_x=waypoints[:, 0],
            waypoints_y=waypoints[:, 1],
            waypoints_psi=waypoints[:, 6]
        )

        self.get_logger().info(f"Lane {lane_index}: loaded {len(waypoints)} pts from {csv_path} (length {track_length:.2f} m)")

        return {
            'index': lane_index,
            'csv_path': csv_path,
            'waypoints': waypoints,
            'track_length': track_length,
            'frenet': frenet_converter,
            'has_speed_column': not missing_speed,
        }

    def apply_lane(self, lane_idx: int):
        """Switch active lane to the given index."""
        if not self.lanes:
            self.get_logger().warn("No lanes loaded; cannot apply lane.")
            return
        lane_idx = int(np.clip(lane_idx, 0, len(self.lanes) - 1))
        lane = self.lanes[lane_idx]
        self.current_lane_idx = lane_idx
        self.waypoint_array_in_map = lane['waypoints']
        self.track_length = lane['track_length']
        self.frenet_converter = lane['frenet']
        self.has_waypoints = True
        self.get_logger().info(f"Active lane set to {lane_idx} ({lane['csv_path']})")
        self._path_published_once = False
        if hasattr(self, 'path_pub'):
            self.publish_global_path()

    def lane_index_callback(self, msg: Int32):
        """Handle lane index updates from the lane selector."""
        if not self.lanes:
            self.get_logger().warn("Received lane command but no lanes are loaded.")
            return

        target_idx = int(msg.data)
        if target_idx < 0 or target_idx >= len(self.lanes):
            self.get_logger().warn(f"Requested lane index {target_idx} out of range (0-{len(self.lanes) - 1}).")
            return

        if target_idx == self.current_lane_idx:
            return

        self.get_logger().info(f"Lane change requested: {self.current_lane_idx} -> {target_idx}")
        self.apply_lane(target_idx)

    def publish_global_path(self):
        """Publish global waypoint path for visualization"""
        if not self.has_waypoints:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        pose_array = PoseArray()
        pose_array.header = path_msg.header

        for wp in self.waypoint_array_in_map:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

            pose_simple = Pose()
            pose_simple.position.x = wp[0]
            pose_simple.position.y = wp[1]
            pose_simple.position.z = 0.0
            yaw = wp[6]
            qx, qy, qz, qw = self._yaw_to_quaternion(yaw)
            pose_simple.orientation.x = qx
            pose_simple.orientation.y = qy
            pose_simple.orientation.z = qz
            pose_simple.orientation.w = qw
            pose_array.poses.append(pose_simple)

        if len(self.waypoint_array_in_map) > 1:
            first = self.waypoint_array_in_map[0]

            closing_pose = PoseStamped()
            closing_pose.header = path_msg.header
            closing_pose.pose.position.x = first[0]
            closing_pose.pose.position.y = first[1]
            closing_pose.pose.position.z = 0.0
            path_msg.poses.append(closing_pose)

            closing_pose_simple = Pose()
            closing_pose_simple.position.x = first[0]
            closing_pose_simple.position.y = first[1]
            closing_pose_simple.position.z = 0.0
            yaw = first[6]
            qx, qy, qz, qw = self._yaw_to_quaternion(yaw)
            closing_pose_simple.orientation.x = qx
            closing_pose_simple.orientation.y = qy
            closing_pose_simple.orientation.z = qz
            closing_pose_simple.orientation.w = qw
            pose_array.poses.append(closing_pose_simple)

        self.path_pub.publish(path_msg)
        if pose_array.poses:
            self.waypoints_pose_pub.publish(pose_array)
        if not self._path_published_once:
            self.get_logger().info("Published global path")
            self._path_published_once = True

    def get_current_pose_from_tf(self):
        """
        Get current pose from TF (map -> base_link transform)
        Returns: (x, y, theta) or None if failed
        """
        try:
            from rclpy.time import Time
            from rclpy.duration import Duration
            # Humble fix: Use rclpy.time.Time() requires arguments
            # Using Time() without args defaults to t=0 which means "latest"
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time().to_msg(),  # Convert to builtin_interfaces.msg.Time
                timeout=Duration(seconds=self.tf_timeout)
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation
            _, _, theta = quat2euler([q.w, q.x, q.y, q.z])

            return (x, y, theta)

        except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup failed: {e}",
                throttle_duration_sec=1.0
            )
            return None

    def get_current_pose_from_amcl(self):
        """
        Get current pose from AMCL topic (fallback)
        Returns: (x, y, theta) or None if failed
        """
        if not self.has_pose:
            return None

        from rclpy.time import Time
        current_time = self.get_clock().now()
        current_msg = current_time.to_msg()

        current_sec = current_msg.sec + current_msg.nanosec / 1e9
        last_sec = self.last_pose_time.sec + self.last_pose_time.nanosec / 1e9
        pose_age = current_sec - last_sec

        if pose_age > self.max_pose_age:
            self.get_logger().warn(
                f"AMCL pose is {pose_age:.2f}s old (max: {self.max_pose_age:.2f}s)",
                throttle_duration_sec=2.0
            )
            return None

        return (
            self.current_pose.pose.pose.position.x,
            self.current_pose.pose.pose.position.y,
            self.current_pose_yaw
        )

    def get_current_pose(self):
        """
        Get current pose (TF first, then AMCL fallback, then odom fallback)
        Returns: (x, y, theta) or None if all methods failed
        """
        # Method 1: TF (real-time, recommended)
        if self.use_tf_for_localization:
            pose = self.get_current_pose_from_tf()
            if pose is not None:
                return pose

            self.get_logger().info(
                "TF failed, falling back to AMCL topic",
                throttle_duration_sec=5.0
            )

        # Method 2: AMCL topic (fallback)
        if self.use_amcl_pose or self.use_tf_for_localization:
            pose = self.get_current_pose_from_amcl()
            if pose is not None:
                return pose

        # Method 3: Odom (last resort)
        if self.use_odom_pose and self.has_pose:
            return (
                self.current_pose.pose.pose.position.x,
                self.current_pose.pose.pose.position.y,
                self.current_pose_yaw
            )

        return None

    def _update_pose(self, x: float, y: float, theta: float):
        """Update stored pose and associated Frenet coordinates."""
        self.position_in_map = np.array([[x, y, theta]])

        if self.has_waypoints and hasattr(self, 'frenet_converter'):
            try:
                s, d = self.frenet_converter.get_frenet(np.array([x]), np.array([y]))

                if self.has_odom:
                    vx = self.speed_now * np.cos(theta)
                    vy = self.speed_now * np.sin(theta)
                    vs, vd = self.frenet_converter.get_frenet_velocities(vx, vy, theta, s[0])
                    self.position_in_map_frenet = np.array([s[0], d[0], vs, vd])
                else:
                    self.position_in_map_frenet = np.array([s[0], d[0], 0.0, 0.0])
            except Exception as e:
                self.get_logger().warn(f"Frenet conversion failed: {e}")

        self.has_pose = True

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """AMCL pose callback - fallback for TF"""
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
        """Odometry callback - velocity and fallback position"""
        self.speed_now = msg.twist.twist.linear.x

        if self.use_odom_pose or not self.has_pose:
            pose = msg.pose.pose
            x = pose.position.x
            y = pose.position.y
            _, _, theta = quat2euler([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
            self._update_pose(x, y, theta)

        # Update Frenet velocities
        if self.has_pose and self.has_waypoints and hasattr(self, 'frenet_converter'):
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
        """IMU callback - acceleration for steering scaling"""
        self.acc_now[1:] = self.acc_now[:-1]
        self.acc_now[0] = msg.linear_acceleration.x

    def nearest_waypoint(self, position):
        """Find index of nearest waypoint to position"""
        if not self.has_waypoints:
            return 0

        position_array = np.array([position] * len(self.waypoint_array_in_map))
        distances = np.linalg.norm(position_array - self.waypoint_array_in_map[:, :2], axis=1)
        return np.argmin(distances)

    def control_loop(self):
        """Main control loop - called at loop_rate Hz"""
        if not self.has_odom or not self.has_waypoints:
            self.get_logger().warn(
                f"Waiting: odom={self.has_odom}, waypoints={self.has_waypoints}",
                throttle_duration_sec=2.0
            )
            return

        # Get current pose from TF or fallback
        pose = self.get_current_pose()
        if pose is None:
            self.get_logger().warn(
                "No valid pose available (TF/AMCL/odom all failed)",
                throttle_duration_sec=1.0
            )
            return

        x, y, theta = pose
        self._update_pose(x, y, theta)

        if not hasattr(self, '_control_active_logged'):
            self.get_logger().info("=== CONTROL LOOP ACTIVE ===")
            if self.use_tf_for_localization:
                self.get_logger().info("Using TF for real-time localization (map -> base_link)")
            elif self.use_amcl_pose:
                self.get_logger().info("Using /amcl_pose topic for localization")
            elif self.use_odom_pose:
                self.get_logger().info("Using /odom topic for localization")
            self._control_active_logged = True

        try:
            speed, acceleration, jerk, steering_angle, L1_point, L1_distance, idx_nearest = \
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

            self.get_logger().info(
                f"Control: pos=({self.position_in_map[0,0]:.2f},{self.position_in_map[0,1]:.2f}), "
                f"lane={self.current_lane_idx}, speed={speed:.2f}, steer={steering_angle:.3f}, "
                f"frenet=({self.position_in_map_frenet[0]:.2f},{self.position_in_map_frenet[1]:.2f})",
                throttle_duration_sec=1.0
            )

            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = self.get_clock().now().to_msg()
            ack_msg.header.frame_id = 'base_link'
            ack_msg.drive.speed = float(speed)
            ack_msg.drive.acceleration = float(acceleration)
            ack_msg.drive.jerk = float(jerk)
            ack_msg.drive.steering_angle = float(steering_angle)

            self.drive_pub.publish(ack_msg)

            if L1_point is not None:
                lookahead_msg = PoseStamped()
                lookahead_msg.header.stamp = ack_msg.header.stamp
                lookahead_msg.header.frame_id = 'map'
                lookahead_msg.pose.position.x = float(L1_point[0])
                lookahead_msg.pose.position.y = float(L1_point[1])
                lookahead_msg.pose.position.z = 0.0
                dx = L1_point[0] - self.position_in_map[0, 0]
                dy = L1_point[1] - self.position_in_map[0, 1]
                yaw = math.atan2(dy, dx)
                qx, qy, qz, qw = self._yaw_to_quaternion(yaw)
                lookahead_msg.pose.orientation.x = qx
                lookahead_msg.pose.orientation.y = qy
                lookahead_msg.pose.orientation.z = qz
                lookahead_msg.pose.orientation.w = qw
                self.lookahead_pub.publish(lookahead_msg)

                lookahead_dist_msg = Float32()
                lookahead_dist_msg.data = float(L1_distance)
                self.lookahead_distance_pub.publish(lookahead_dist_msg)

        except Exception as e:
            import traceback
            self.get_logger().error(f"Control loop error: {e}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")


def main(args=None):
    rclpy.init(args=args)
    controller = ControllerManager()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
