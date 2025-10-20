include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",

  -- 외부 /odom 사용
  provide_odom_frame = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  publish_frame_projected_to_2d = true,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.005,
  trajectory_publish_period_sec = 0.03,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- 2D 트래젝터리 빌더
TRAJECTORY_BUILDER_2D.use_imu_data = true            -- 브릿지에서 gyro/accel 제공
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 3.0

-- 라이다 필터/범위
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.5
TRAJECTORY_BUILDER_2D.min_z = -0.2
TRAJECTORY_BUILDER_2D.max_z = 0.5

-- 스캔 매칭 (하나만 유지)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight    = 400.0

-- 모션 필터
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.05

-- 서브맵/해상도
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- 포즈 그래프 최적화(주기/가중치)
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 5e2
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight    = 5e2

-- 루프 클로저
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 12.0
POSE_GRAPH.constraint_builder.min_score = 0.85
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.9

return options
