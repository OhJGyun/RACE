#!/usr/bin/env python3
"""
MAP Controller Manager - Ported from race_stack to ROS2
Original: https://github.com/ForzaETH/race_stack
Adapted for f1tenth_miru3 topics and ROS2

Key adaptations:
- ROS2 API (rclpy instead of rospy)
- f1tenth_miru3 topics (/amcl_pose, /odom instead of /car_state/pose, /car_state/odom)
- Simplified: no MPC, no FTG, no trailing - pure MAP controller only
- Waypoints from CSV file instead of /local_waypoints topic
- Minimal frenet coordinate support
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
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int32, Int32MultiArray  # 레인 세그먼트 정보를 받기 위한 Int32MultiArray 사용
from transforms3d.euler import quat2euler
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException, Buffer, TransformListener
import csv

from .map_controller import MAP_Controller
from .frenet_converter import FrenetConverter


class ControllerManager(Node):
    """
    MAP Controller Manager for f1tenth_miru3

    Subscribes to:
    - /amcl_pose: ego car position (x, y, theta) in map frame (REAL CAR)
    - /ego_racecar/odom: ego car velocity (SIM) or /odom (REAL CAR)
    - /imu/data: acceleration for steering scaling (optional)

    Publishes to:
    - /drive: Ackermann drive commands (REAL CAR compatible)
    - /map_controller/lookahead_point: L1 lookahead point visualization
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
        self.position_in_map_frenet = np.array([0.0, 0.0, 0.0, 0.0])  # [s, d, vs, vd] - simplified
        self.speed_now = 0.0
        self.acc_now = np.zeros(5)  # last 5 acceleration values
        self.waypoint_array_in_map = None
        self.track_length = 0.0

        # Multi-lane support
        self.lane_waypoints = []  # List of all lane waypoint arrays
        self.current_lane_idx = 0  # Currently active lane index
        self.lane_change_active = False  # Flag for lane change in progress
        self.lane_change_start_time = None  # Time when lane change started
        self.lane_change_duration = 1.0  # Duration to apply reduced LD (seconds)
        # 레인별 Frenet 변환기 캐시 (세그먼트만 써도 s/d는 전체 레인 기준으로 일관 유지)
        self._lane_frenet_converters = []

        # Control state
        self.state = "RACING"  # Simplified: no state machine, always racing
        self.opponent = None  # No opponent tracking in f1tenth_miru3

        # Flags
        self.has_pose = False
        self.has_odom = False
        self.has_waypoints = False

        # For AMCL fallback
        self.current_pose = None
        self.current_pose_yaw = 0.0
        # Store last pose time as builtin_interfaces.msg.Time (from messages).
        # Use RclTime.from_msg for time arithmetic in Humble.
        self.last_pose_time = self.get_clock().now().to_msg()

        # Load waypoints from CSV
        self.load_waypoints_from_csv()

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

            prioritize_dyn=False,  # No opponent tracking
            trailing_gap=0.0,
            trailing_p_gain=0.0,
            trailing_i_gain=0.0,
            trailing_d_gain=0.0,
            blind_trailing_speed=0.0,

            loop_rate=self.loop_rate,
            LUT_name=self.LUT_name,
            state_machine_rate=self.loop_rate,
            lat_accel_max=self.lat_accel_max,  # 곡률 기반 v_max 제한 활성화 시 사용될 최대 횡가속 파라미터

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

        # Subscribe to AMCL (as fallback for TF or primary if not using TF)
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

        # Optional IMU for acceleration (if available)
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile=qos_sensor
        )
        self.get_logger().info(f"Subscribed to {self.imu_topic} for acceleration (optional)")

        # Lane selector subscription (if multi-lane enabled)
        if self.lane_csv_paths and len(self.lane_csv_paths) > 1:
            self.lane_selector_sub = self.create_subscription(
                Int32,
                self.lane_selector_topic,
                self.lane_selector_callback,
                10
            )
            self.get_logger().info(f"Subscribed to {self.lane_selector_topic} for lane switching")

            # 레인 세그먼트(로컬 윈도우) 갱신 토픽 구독
            self.lane_segment_sub = self.create_subscription(
                Int32MultiArray,              # data = [lane_id, start_idx, end_idx)
                self.lane_segment_topic,      # lane_selector에서 퍼블리시하는 세그먼트 토픽
                self.lane_segment_callback,   # 세그먼트 수신 콜백: 활성 웨이포인트 덩어리를 갱신
                10,                           # 큐 크기
            )
            self.get_logger().info(f"Subscribed to {self.lane_segment_topic} for local segments")

        # Control loop timer
        self.timer = self.create_timer(1.0 / self.loop_rate, self.control_loop)

        # Publish global path once and keep re-publishing for visualization
        self.publish_global_path()
        self.path_timer = self.create_timer(1.0, self.publish_global_path)

        self.get_logger().info("MAP Controller initialized! Waiting for pose and odometry...")

    @staticmethod
    def _yaw_to_quaternion(yaw: float):
        half = 0.5 * yaw
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def declare_ros_parameters(self):
        """Declare all ROS2 parameters"""
        # CSV file path
        self.declare_parameter('csv_file_path', '')

        # Multi-lane support
        self.declare_parameter('lane_csv_paths', [''])  # List of lane CSV paths (empty string for type inference)
        self.declare_parameter('lane_selector_topic', '/lane_selector/target_lane')
        self.declare_parameter('lane_segment_topic', 'lane_selector/current_segment')  # 세그먼트 수신 토픽 이름
        self.declare_parameter('lane_change_ld_gain', 0.7)  # LD multiplier during lane change (0~1)
        self.declare_parameter('lane_change_speed_gain', 0.7)  # Speed multiplier during lane change (0~1)
        # 로컬 세그먼트 크롭: 차량 인덱스 기준 양쪽 점 개수 (총 2*n 사용)
        self.declare_parameter('segment_points_each_side', 40)
        # 곡률 기반 속도 제한을 위한 최대 허용 횡가속 [m/s^2]
        self.declare_parameter('lat_accel_max', 4.0)  # 헤딩/곡률 기반 감속 중 곡률 v_max 제한을 사용할 때 상한 가속도

        # L1 controller parameters (matching race_stack)
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

        # Steering lookup table name
        self.declare_parameter('steering_lut', '')  # Empty = use kinematic fallback

        # Loop rate
        self.declare_parameter('loop_rate_hz', 40.0)

        # Real car settings: TF for real-time localization (like pure_pursuit)
        self.declare_parameter('use_tf_for_localization', True)  # True = use TF (REAL CAR)
        self.declare_parameter('tf_timeout', 0.1)  # TF lookup timeout (seconds)
        self.declare_parameter('max_pose_age', 0.5)  # Max age for fallback amcl_pose (seconds)

        # Legacy fallback options
        self.declare_parameter('use_odom_pose', False)  # Fallback: use odom pose
        self.declare_parameter('use_amcl_pose', False)  # Fallback: use AMCL pose topic

        # Topic names (configurable for real car vs sim)
        self.declare_parameter('odom_topic', '/odom')  # /odom for real car, /ego_racecar/odom for sim
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('drive_topic', '/drive')

    def load_parameters(self):
        """Load all parameters from ROS2 parameter server"""
        self.csv_file_path = self.get_parameter('csv_file_path').value

        # Multi-lane support
        self.lane_csv_paths = self.get_parameter('lane_csv_paths').value
        self.lane_selector_topic = self.get_parameter('lane_selector_topic').value
        self.lane_segment_topic = self.get_parameter('lane_segment_topic').value  # 세그먼트 토픽
        self.lane_change_ld_gain = self.get_parameter('lane_change_ld_gain').value
        self.lane_change_speed_gain = self.get_parameter('lane_change_speed_gain').value
        self.segment_points_each_side = int(self.get_parameter('segment_points_each_side').value)  # 세그먼트 크기(한쪽)
        self.lat_accel_max = float(self.get_parameter('lat_accel_max').value)  # 곡률 기반 v_max 제한용 (현재 코드는 비활성 상태)

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

    def load_waypoints_from_csv(self):
        """Load waypoints from CSV file(s)
        CSV format: x, y, speed, [optional: d, s, kappa, psi]
        Minimal format: x, y, speed

        Supports multi-lane: if lane_csv_paths is specified, load all lanes
        """
        # Load multi-lane waypoints if available
        if self.lane_csv_paths and len(self.lane_csv_paths) > 0:
            self.get_logger().info(f"Loading {len(self.lane_csv_paths)} lane(s)...")
            for idx, lane_path in enumerate(self.lane_csv_paths):
                waypoints = self._load_single_waypoint_file(lane_path)
                if waypoints is not None:
                    self.lane_waypoints.append(waypoints)
                    self.get_logger().info(f"  Lane {idx}: {lane_path} ({len(waypoints)} waypoints)")

            if not self.lane_waypoints:
                self.get_logger().error("Failed to load any lanes!")
                return

            # Set default lane to first one
            self.waypoint_array_in_map = self.lane_waypoints[0]
            self.current_lane_idx = 0
            self.track_length = self.waypoint_array_in_map[-1, 4]
            self.has_waypoints = True
            # 세그먼트 모드에서도 s/d 일관성을 위해 레인별 Frenet 변환기 생성
            self._lane_frenet_converters = []
            for idx, wp in enumerate(self.lane_waypoints):
                try:
                    conv = FrenetConverter(
                        waypoints_x=wp[:, 0],
                        waypoints_y=wp[:, 1],
                        waypoints_psi=wp[:, 6],
                    )
                    self._lane_frenet_converters.append(conv)  # 생성 성공 시 목록에 추가
                except Exception as e:
                    self.get_logger().warn(f"Frenet converter init failed for lane {idx}: {e}")
                    self._lane_frenet_converters.append(None)  # 실패 시 자리 채우기
            # 현재 활성 레인의 Frenet 변환기로 설정
            if self._lane_frenet_converters and self._lane_frenet_converters[0] is not None:
                self.frenet_converter = self._lane_frenet_converters[0]
            self.get_logger().info(f"✅ Multi-lane initialized: {len(self.lane_waypoints)} lanes, default lane 0")
            return

        # Fallback to single CSV file
        if not self.csv_file_path:
            self.get_logger().error("No CSV file path specified!")
            return

        waypoints = []
        try:
            with open(self.csv_file_path, 'r') as f:
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
                        speed = float(row[2]) if len(row) > 2 else 2.0
                    except ValueError:
                        # Skip rows that can't be converted to float (headers, comments)
                        continue

                    if waypoints and abs(waypoints[-1][0] - x) < 1e-4 and abs(waypoints[-1][1] - y) < 1e-4:
                        continue

                    # For race_stack compatibility, need: [x, y, v, d, s, kappa, psi, ax]
                    # We'll compute minimal values
                    d = 0.0  # Assume on centerline
                    s = 0.0  # Will compute cumulative distance
                    kappa = 0.0  # Will compute curvature
                    psi = 0.0  # Will compute heading
                    ax = 0.0  # No acceleration planning

                    waypoints.append([x, y, speed, d, s, kappa, psi, ax])

            if len(waypoints) < 2:
                self.get_logger().error(f"Not enough waypoints in {self.csv_file_path}")
                return

            # Post-process waypoints
            waypoints = np.array(waypoints)

            # Compute cumulative distance (s)
            for i in range(1, len(waypoints)):
                dx = waypoints[i, 0] - waypoints[i-1, 0]
                dy = waypoints[i, 1] - waypoints[i-1, 1]
                waypoints[i, 4] = waypoints[i-1, 4] + np.sqrt(dx**2 + dy**2)

            self.track_length = waypoints[-1, 4]

            # Remove duplicate closing point if it matches the start point
            if len(waypoints) > 1 and abs(waypoints[0, 0] - waypoints[-1, 0]) < 1e-4 and abs(waypoints[0, 1] - waypoints[-1, 1]) < 1e-4:
                waypoints = waypoints[:-1]
                self.track_length = waypoints[-1, 4]

            # Compute heading (psi)
            for i in range(len(waypoints) - 1):
                dx = waypoints[i+1, 0] - waypoints[i, 0]
                dy = waypoints[i+1, 1] - waypoints[i, 1]
                waypoints[i, 6] = np.arctan2(dy, dx)
            waypoints[-1, 6] = waypoints[-2, 6]  # Last waypoint same as previous

            # Compute curvature (kappa) - simplified
            for i in range(1, len(waypoints) - 1):
                psi_prev = waypoints[i-1, 6]
                psi_next = waypoints[i+1, 6]
                ds = waypoints[i+1, 4] - waypoints[i-1, 4]
                if ds > 0:
                    waypoints[i, 5] = (psi_next - psi_prev) / ds

            self.waypoint_array_in_map = waypoints
            self.has_waypoints = True

            # Initialize Frenet converter (race_stack style)
            self.frenet_converter = FrenetConverter(
                waypoints_x=waypoints[:, 0],
                waypoints_y=waypoints[:, 1],
                waypoints_psi=waypoints[:, 6]
            )

            self.get_logger().info(f"Loaded {len(waypoints)} waypoints from {self.csv_file_path}")
            self.get_logger().info(f"Track length: {self.track_length:.2f} m")
            self.get_logger().info("Initialized Frenet converter")

        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")

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

        # 세그먼트 모드(track_length == 0.0)에서는 닫힘선을 그리지 않음
        # 전체 랩 경로(폐곡선)일 때만 시작점을 다시 추가하여 시각적으로 닫힌 경로 표시
        if len(self.waypoint_array_in_map) > 1 and self.track_length > 0.0:
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
            # Lookup transform from map to base_link using explicit rclpy Time/Duration (Humble)
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                RclTime(),  # latest available transform
                timeout=Duration(seconds=self.tf_timeout)
            )

            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Convert quaternion to yaw
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

        # Check data freshness using rclpy Time/Duration (Humble style)
        current_time = self.get_clock().now()
        last_time = RclTime.from_msg(self.last_pose_time)
        pose_age = (current_time - last_time).nanoseconds / 1e9

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
        # Method 1: TF (real-time, recommended for real car)
        if self.use_tf_for_localization:
            pose = self.get_current_pose_from_tf()
            if pose is not None:
                return pose

            # TF failed, try AMCL fallback
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

        # Proper Frenet conversion using race_stack FrenetConverter
        if self.has_waypoints and hasattr(self, 'frenet_converter'):
            try:
                # Get Frenet coordinates (s, d)
                s, d = self.frenet_converter.get_frenet(np.array([x]), np.array([y]))

                # Get Frenet velocities (vs, vd) if we have odom data
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
        """AMCL pose callback - ego car position in map frame (fallback for TF)"""
        # Store for fallback
        self.current_pose = msg
        orientation = msg.pose.pose.orientation
        # transforms3d uses (w, x, y, z) order
        _, _, self.current_pose_yaw = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z])
        self.last_pose_time = msg.header.stamp

        # Only update position if not using TF
        if not self.use_tf_for_localization:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self._update_pose(x, y, self.current_pose_yaw)
        else:
            self.has_pose = True  # Mark that we have AMCL data available for fallback

    def odom_callback(self, msg: Odometry):
        """Odometry callback - ego car velocity (race_stack style)"""
        self.speed_now = msg.twist.twist.linear.x

        if self.use_odom_pose or not self.has_pose:
            pose = msg.pose.pose
            x = pose.position.x
            y = pose.position.y
            _, _, theta = quat2euler([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
            self._update_pose(x, y, theta)

        # Update Frenet velocities when odom arrives
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
                pass  # Silent fail, not critical

        self.has_odom = True

    def imu_callback(self, msg: Imu):
        """IMU callback - acceleration for steering scaling"""
        # Shift acceleration history
        self.acc_now[1:] = self.acc_now[:-1]
        self.acc_now[0] = msg.linear_acceleration.x  # Longitudinal acceleration

    def lane_selector_callback(self, msg: Int32):
        """Lane selector callback - switch to selected lane"""
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
        self.track_length = self.waypoint_array_in_map[-1, 4]
        # 레인 변경 시, 프레네 변환기도 해당 레인의 전체 경로 기준으로 교체
        if 0 <= desired_lane < len(self._lane_frenet_converters):
            if self._lane_frenet_converters[desired_lane] is not None:
                self.frenet_converter = self._lane_frenet_converters[desired_lane]

        # Activate lane change mode: apply reduced LD and speed for better tracking
        self.lane_change_active = True
        self.lane_change_start_time = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            f"🛣️ Lane switched: {prev_lane} → {desired_lane} ({len(self.waypoint_array_in_map)} waypoints), "
            f"LD×{self.lane_change_ld_gain:.2f}, Speed×{self.lane_change_speed_gain:.2f} for {self.lane_change_duration:.1f}s"
        )

    def lane_segment_callback(self, msg: Int32MultiArray):
        """lane_selector에서 퍼블리시한 로컬 세그먼트 갱신 처리
        msg.data: [lane_id, start_idx, end_idx) 3개 인덱스 사용
        """
        if not self.lane_waypoints:
            return
        if not msg.data or len(msg.data) < 3:
            return
        lane_id = int(msg.data[0])
        start_idx = int(msg.data[1])
        end_idx = int(msg.data[2])

        if lane_id < 0 or lane_id >= len(self.lane_waypoints):  # 유효한 레인 인덱스인지 검사
            self.get_logger().warn(f"[SEGMENT] invalid lane_id {lane_id}")
            return

        full = self.lane_waypoints[lane_id]
        n = len(full)
        if n == 0:  # 빈 레인이면 무시
            return
        # 인덱스 클램프 (경계 밖 방지)
        start_idx = max(0, min(start_idx, n - 1))
        end_idx = max(start_idx + 1, min(end_idx, n))

        # 필요 시 중앙(≈차량 위치) 기준으로 고정 크기 윈도우로 추가 크롭
        if self.segment_points_each_side and (end_idx - start_idx) > 2 * self.segment_points_each_side:
            mid = (start_idx + end_idx - 1) // 2
            new_start = max(0, mid - self.segment_points_each_side)
            new_end = min(n, mid + self.segment_points_each_side)
            start_idx, end_idx = new_start, new_end

        segment = full[start_idx:end_idx]
        if len(segment) < 2:  # 최소 2점 이상 필요
            return

        # 레인이 바뀌었으면 활성 레인 교체 및 Frenet 변환기 동기화
        if lane_id != self.current_lane_idx:
            self.current_lane_idx = lane_id
            # s/d 일관성을 위해 전체 레인 기준 변환기로 교체
            if 0 <= lane_id < len(self._lane_frenet_converters):
                if self._lane_frenet_converters[lane_id] is not None:
                    self.frenet_converter = self._lane_frenet_converters[lane_id]

        # 제어/시각화에는 로컬 세그먼트만 사용
        self.waypoint_array_in_map = segment
        # 세그먼트 모드에서는 s 기반 탐색 대신 인덱스 기반 L1 계산을 쓰도록 트랙 길이를 0으로 설정
        self.track_length = 0.0

        # 웨이포인트 사용 가능 플래그 활성화
        self.has_waypoints = True
        # 로그(스로틀)
        self.get_logger().info(
            f"[SEGMENT] lane={lane_id}, segment=[{start_idx}, {end_idx}) ({len(segment)} pts)",
            throttle_duration_sec=1.0,
        )

    def _load_single_waypoint_file(self, csv_path: str):
        """Load waypoints from a single CSV file
        Returns processed numpy array or None if failed
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
                        speed = float(row[2]) if len(row) > 2 else 2.0
                    except ValueError:
                        # Skip rows that can't be converted to float (headers, comments)
                        continue

                    if waypoints and abs(waypoints[-1][0] - x) < 1e-4 and abs(waypoints[-1][1] - y) < 1e-4:
                        continue

                    # For race_stack compatibility: [x, y, v, d, s, kappa, psi, ax]
                    waypoints.append([x, y, speed, 0.0, 0.0, 0.0, 0.0, 0.0])

            if len(waypoints) < 2:
                self.get_logger().error(f"Not enough waypoints in {csv_path}")
                return None

            # Post-process waypoints
            waypoints = np.array(waypoints)

            # Compute cumulative distance (s)
            for i in range(1, len(waypoints)):
                dx = waypoints[i, 0] - waypoints[i-1, 0]
                dy = waypoints[i, 1] - waypoints[i-1, 1]
                waypoints[i, 4] = waypoints[i-1, 4] + np.sqrt(dx*dx + dy*dy)

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

            return waypoints

        except Exception as e:
            self.get_logger().error(f"Failed to load {csv_path}: {e}")
            return None

    def nearest_waypoint(self, position):
        """Find index of nearest waypoint to position"""
        if not self.has_waypoints:
            return 0

        position_array = np.array([position] * len(self.waypoint_array_in_map))
        distances = np.linalg.norm(position_array - self.waypoint_array_in_map[:, :2], axis=1)
        return np.argmin(distances)

    def control_loop(self):
        """Main control loop - called at loop_rate Hz"""
        # Wait for initialization (odom and waypoints required, pose will be from TF)
        if not self.has_odom or not self.has_waypoints:
            self.get_logger().warn(
                f"Waiting: odom={self.has_odom}, waypoints={self.has_waypoints}",
                throttle_duration_sec=2.0
            )
            return

        # Get current pose from TF (real-time) or fallback to AMCL/odom
        pose = self.get_current_pose()
        if pose is None:
            self.get_logger().warn(
                "No valid pose available (TF/AMCL/odom all failed)",
                throttle_duration_sec=1.0
            )
            return

        # Update position with real-time pose from TF
        x, y, theta = pose
        self._update_pose(x, y, theta)

        # Log that control loop is active (once)
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
            # Check if lane change duration has elapsed
            if self.lane_change_active:
                current_time = self.get_clock().now().nanoseconds * 1e-9
                elapsed_time = current_time - self.lane_change_start_time
                if elapsed_time >= self.lane_change_duration:
                    # Lane change period ended, restore original LD parameters
                    self.lane_change_active = False
                    self.map_controller.m_l1 = self.m_l1
                    self.map_controller.q_l1 = self.q_l1
                    self.get_logger().info("🛣️ Lane change complete, LD restored to normal")

            # Apply reduced LD during lane change
            if self.lane_change_active:
                # Temporarily reduce LD for better lane tracking
                self.map_controller.m_l1 = self.m_l1 * self.lane_change_ld_gain
                self.map_controller.q_l1 = self.q_l1 * self.lane_change_ld_gain
            else:
                # Ensure normal LD parameters
                self.map_controller.m_l1 = self.m_l1
                self.map_controller.q_l1 = self.q_l1

            # Call MAP controller main_loop (matching race_stack API)
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

            # Apply speed gain during lane change
            if self.lane_change_active:
                speed = speed * self.lane_change_speed_gain
                acceleration = acceleration * self.lane_change_speed_gain
                # jerk는 변화율이므로 gain^2 적용
                jerk = jerk * (self.lane_change_speed_gain ** 2)

            # Debug output
            self.get_logger().info(
                f"Control: pos=({self.position_in_map[0,0]:.2f},{self.position_in_map[0,1]:.2f}), "
                f"speed={speed:.2f}{'*' if self.lane_change_active else ''}, steer={steering_angle:.3f}, "
                f"frenet=({self.position_in_map_frenet[0]:.2f},{self.position_in_map_frenet[1]:.2f})",
                throttle_duration_sec=1.0
            )

            # Create and publish Ackermann command
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
