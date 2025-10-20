include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = true,
  use_nav_sat = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.,  -- IMU 완전 비활성화
  use_landmarks = false,
  publish_frame_projected_to_2d = false,
  landmarks_sampling_ratio = 1.,
}

-- IMU 완전 비활성화, 오돔+스캔매칭만 사용
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 0.0

-- 오돔 기반 초기 추정은 options 테이블에서 use_odometry = true로 설정됨

-- 스캔 매칭 최적화
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres 스캔 매처 가중치 조정 (오돔 없이도 안정적)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 50.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20

-- 맵 빌더 설정
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.num_background_threads = 4

-- 라이다 범위 설정
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.5
TRAJECTORY_BUILDER_2D.min_z = -0.3
TRAJECTORY_BUILDER_2D.max_z = 0.3

-- 모션 필터 (오돔 기반으로 조정)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(3.0)

-- might be able to optimize these parameters
-- see: http://google-cartographer-ros.readthedocs.io/en/latest/tuning.html
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80
POSE_GRAPH.optimize_every_n_nodes = 30

-- 제약점 샘플링 비율 ↑ (더 많은 제약 확보)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.max_constraint_distance = 12.0
POSE_GRAPH.constraint_builder.min_score            = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
