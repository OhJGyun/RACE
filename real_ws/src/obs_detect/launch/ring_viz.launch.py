from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    scan_topic   = LaunchConfiguration('scan_topic')
    frame_id     = LaunchConfiguration('marker_frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('marker_frame_id', default_value='map'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='obs_detect',
            executable='ring_viz_node',  # simple_scan_viz.py의 실행 파일명
            name='ring_viz_node',
            output='screen',
            parameters=[{
                'scan_topic': scan_topic,
                'marker_frame_id': frame_id,
                'use_sim_time': use_sim_time,
                # -------------------------
                # ✅ DBSCAN 관련 파라미터 추가
                # -------------------------
                'db_eps': 0.25,           # 클러스터 반경 [m]
                'db_min_samples': 5,      # 최소 점 개수
                'point_scale': 0.04,      # RViz 점 크기
                'show_centers': True,     # 중심점 표시 여부
                'tf_timeout': 0.3,        # TF timeout
            }],
        ),
    ])
