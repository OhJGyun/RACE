from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for real-vehicle LiDAR obstacle detection node.
    - Uses TF2 (map→base_link) for localization
    - Publishes visualization markers to /scan_viz/markers
    """

    # -------------------------------------------------------
    # Declare configurable arguments
    # -------------------------------------------------------
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScan topic name (LiDAR input)'
    )

    outer_bound_csv_arg = DeclareLaunchArgument(
        'outer_csv',
        default_value='/home/ircv7/RACE/bound/1030/outer_bound_world.csv',
        description='Path to outer boundary CSV file'
    )

    inner_bound_csv_arg = DeclareLaunchArgument(
        'inner_csvs',
        default_value='/home/ircv7/RACE/bound/1030/inner_bound_world.csv',
        description='Path to inner boundary CSV (can be multiple)'
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame_id',
        default_value='map',
        description='Frame ID for visualization markers (usually "map")'
    )

    roi_deg_arg = DeclareLaunchArgument(
        'roi_deg',
        default_value='90.0',
        description='Field of view (deg) for obstacle detection'
    )

    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='12.0',
        description='Maximum detection range (m)'
    )

    cluster_epsilon_arg = DeclareLaunchArgument(
        'cluster_epsilon',
        default_value='0.35',
        description='Distance threshold for clustering (m)'
    )

    cluster_min_points_arg = DeclareLaunchArgument(
        'cluster_min_points',
        default_value='4',
        description='Minimum points to form a cluster'
    )

    cluster_confirm_points_arg = DeclareLaunchArgument(
        'cluster_confirm_points',
        default_value='6',
        description='Points required to confirm cluster as obstacle'
    )

    boundary_margin_arg = DeclareLaunchArgument(
        'boundary_margin',
        default_value='0.50',
        description='Margin near map boundaries to ignore detections (m)'
    )

    marker_scale_arg = DeclareLaunchArgument(
        'marker_scale',
        default_value='0.35',
        description='Sphere scale for cluster centers visualization'
    )

    tf_timeout_arg = DeclareLaunchArgument(
        'tf_timeout',
        default_value='0.3',
        description='TF2 lookup timeout (seconds)'
    )

    # -------------------------------------------------------
    # LaunchConfigurations
    # -------------------------------------------------------
    scan_topic = LaunchConfiguration('scan_topic')
    outer_csv = LaunchConfiguration('outer_csv')
    inner_csvs = LaunchConfiguration('inner_csvs')
    marker_frame_id = LaunchConfiguration('marker_frame_id')
    roi_deg = LaunchConfiguration('roi_deg')
    range_max = LaunchConfiguration('range_max')
    cluster_epsilon = LaunchConfiguration('cluster_epsilon')
    cluster_min_points = LaunchConfiguration('cluster_min_points')
    cluster_confirm_points = LaunchConfiguration('cluster_confirm_points')
    boundary_margin = LaunchConfiguration('boundary_margin')
    marker_scale = LaunchConfiguration('marker_scale')
    tf_timeout = LaunchConfiguration('tf_timeout')

    # -------------------------------------------------------
    # Node definition
    # -------------------------------------------------------
    obs_detect_real_node = Node(
        package='obs_detect',
        executable='obs_detect_real_node',
        name='obs_detect_real_node',
        output='screen',
        parameters=[{
            'scan_topic': scan_topic,
            'outer_csv': outer_csv,
            'inner_csvs': inner_csvs,
            'marker_frame_id': marker_frame_id,
            'roi_deg': roi_deg,
            'range_max': range_max,
            'cluster_epsilon': cluster_epsilon,
            'cluster_min_points': cluster_min_points,
            'cluster_confirm_points': cluster_confirm_points,
            'boundary_margin': boundary_margin,
            'marker_scale': marker_scale,
            'tf_timeout': tf_timeout,
        }]
    )

    # -------------------------------------------------------
    # Final launch description
    # -------------------------------------------------------
    return LaunchDescription([
        scan_topic_arg,
        outer_bound_csv_arg,
        inner_bound_csv_arg,
        marker_frame_arg,
        roi_deg_arg,
        range_max_arg,
        cluster_epsilon_arg,
        cluster_min_points_arg,
        cluster_confirm_points_arg,
        boundary_margin_arg,
        marker_scale_arg,
        tf_timeout_arg,
        obs_detect_real_node,
    ])
