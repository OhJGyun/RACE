from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Simplified launch for ring_viz_node (SimpleScanViz)
    모든 파라미터를 코드 내부에서 직접 지정함.
    """

    ring_viz_node = Node(
        package='obs_detect',
        executable='ring_viz_node',
        name='ring_viz_node',
        output='screen',
        parameters=[{
            # 기본
            'scan_topic': '/scan',
            'marker_frame_id': 'map',
            'tf_timeout': 0.3,

            # DBSCAN
            'db_eps': 0.25,
            'db_min_samples': 5,

            # ROI 거리 범위 (m)
            'roi_min_dist': 0.3,
            'roi_max_dist': 6.0,

            # 시각화
            'center_scale': 0.12,

            # 시야각(FOV)
            'fov_deg': 120.0,         # 전방 총 120°
            'fov_center_deg': 0.0,    # 정면 기준 오프셋 (우측 양수)

            # 경계 CSV (map 프레임 기준)
            'outer_bound_csv': '/home/ircv7/RACE/bound/1030/outer_bound_world.csv',
            'inner_bound_csv': '/home/ircv7/RACE/bound/1030/inner_bound_world.csv',
        }],
        remappings=[
            # 필요 시 토픽 리맵 가능
            # ('/scan', '/lidar_front/scan'),
        ]
    )

    return LaunchDescription([ring_viz_node])
