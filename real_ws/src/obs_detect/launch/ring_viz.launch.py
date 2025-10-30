from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Args
    scan_topic         = LaunchConfiguration('scan_topic')
    use_sim_time       = LaunchConfiguration('use_sim_time')
    params_file        = LaunchConfiguration('params_file')

    return LaunchDescription([
        # --- Launch args ---
        DeclareLaunchArgument('scan_topic',   default_value='/scan'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file',  default_value=''),

        # --- Node ---
        Node(
            package='obs_detect',
            executable='ring_viz_node',          # setup.py / CMake의 executable 이름
            name='ring_viz_node',
            output='screen',
            parameters=[
                # 외부 YAML이 있으면 우선 적용
                params_file,
                {
                    'scan_topic': scan_topic,
                    'use_sim_time': use_sim_time,
                    # 안전한 기본값 (YAML에서 덮어쓰기 가능)
                    'target_radius_m': 0.01,         # 1 cm
                    'band_m': 0.002,                 # ±2 mm
                    'tf_timeout': 0.1,
                    'tf_time_tolerance': 0.05,
                    'marker_frame_id': 'map',
                    'point_scale': 0.04,
                    'line_width': 0.02,
                    'strict_tf': True,
                }
            ],
            remappings=[
                # 필요 시 스캔 토픽 리맵 (예: /scan → /sick/scan)
                # ('/scan', scan_topic),  # 노드 내부에서 파라미터로 읽으므로 보통 불필요
            ],
        ),
    ])
