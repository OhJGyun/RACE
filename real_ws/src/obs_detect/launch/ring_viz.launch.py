# simple_scan_viz.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # --------- Launch Args (필요시 마음껏 추가/수정) ---------
    scan_topic        = DeclareLaunchArgument('scan_topic',        default_value=TextSubstitution(text='/scan'))
    marker_frame_id   = DeclareLaunchArgument('marker_frame_id',   default_value=TextSubstitution(text='map'))
    tf_timeout        = DeclareLaunchArgument('tf_timeout',        default_value=TextSubstitution(text='0.3'))

    # 시야/ROI
    fov_deg           = DeclareLaunchArgument('fov_deg',           default_value=TextSubstitution(text='120.0'))
    fov_center_deg    = DeclareLaunchArgument('fov_center_deg',    default_value=TextSubstitution(text='0.0'))
    roi_min_dist      = DeclareLaunchArgument('roi_min_dist',      default_value=TextSubstitution(text='0.20'))
    roi_max_dist      = DeclareLaunchArgument('roi_max_dist',      default_value=TextSubstitution(text='6.00'))

    # 클러스터링/표시
    db_min_samples    = DeclareLaunchArgument('db_min_samples',    default_value=TextSubstitution(text='5'))
    center_scale      = DeclareLaunchArgument('center_scale',      default_value=TextSubstitution(text='0.12'))

    # 링(경계) CSV
    outer_bound_csv   = DeclareLaunchArgument('outer_bound_csv',   default_value=TextSubstitution(text=''))
    inner_bound_csv   = DeclareLaunchArgument('inner_bound_csv',   default_value=TextSubstitution(text=''))

    # 분류(윈도우/임계/반경)
    window_sec        = DeclareLaunchArgument('window_sec',        default_value=TextSubstitution(text='1.0'))   # W >= Td 권장
    lidar_hz          = DeclareLaunchArgument('lidar_hz',          default_value=TextSubstitution(text='40.0'))
    Ts                = DeclareLaunchArgument('Ts',                default_value=TextSubstitution(text='0.1'))   # 정적 커버리지(s)
    Td                = DeclareLaunchArgument('Td',                default_value=TextSubstitution(text='0.5'))   # 동적 커버리지(s)
    static_radius     = DeclareLaunchArgument('static_radius',     default_value=TextSubstitution(text='1.0'))   # Rs (m)
    dynamic_radius    = DeclareLaunchArgument('dynamic_radius',    default_value=TextSubstitution(text='2.0'))   # Rd (m)

    # --------- Node ---------
    node = Node(
        package='YOUR_PKG_NAME',            # <- 패키지명으로 교체
        executable='simple_scan_viz',       # <- setup.py entry_point 또는 설치된 실행파일명
        name='simple_scan_viz',
        output='screen',
        parameters=[{
            'scan_topic':        LaunchConfiguration('scan_topic'),
            'marker_frame_id':   LaunchConfiguration('marker_frame_id'),
            'tf_timeout':        LaunchConfiguration('tf_timeout'),
            'fov_deg':           LaunchConfiguration('fov_deg'),
            'fov_center_deg':    LaunchConfiguration('fov_center_deg'),
            'roi_min_dist':      LaunchConfiguration('roi_min_dist'),
            'roi_max_dist':      LaunchConfiguration('roi_max_dist'),
            'db_min_samples':    LaunchConfiguration('db_min_samples'),
            'center_scale':      LaunchConfiguration('center_scale'),
            'outer_bound_csv':   LaunchConfiguration('outer_bound_csv'),
            'inner_bound_csv':   LaunchConfiguration('inner_bound_csv'),
            'window_sec':        LaunchConfiguration('window_sec'),
            'lidar_hz':          LaunchConfiguration('lidar_hz'),
            'Ts':                LaunchConfiguration('Ts'),
            'Td':                LaunchConfiguration('Td'),
            'static_radius':     LaunchConfiguration('static_radius'),
            'dynamic_radius':    LaunchConfiguration('dynamic_radius'),
        }],
        # remappings=[('/scan', LaunchConfiguration('scan_topic'))],  # 필요시 사용
    )

    return LaunchDescription([
        scan_topic, marker_frame_id, tf_timeout,
        fov_deg, fov_center_deg, roi_min_dist, roi_max_dist,
        db_min_samples, center_scale,
        outer_bound_csv, inner_bound_csv,
        window_sec, lidar_hz, Ts, Td, static_radius, dynamic_radius,
        node
    ])
