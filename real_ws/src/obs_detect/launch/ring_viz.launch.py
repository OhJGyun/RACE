from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for ring_viz_node (SimpleScanViz)
    Visualizes LaserScan clusters as sphere markers in RViz
    """

    # Launch arguments
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScan topic to subscribe to'
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame_id',
        default_value='map',
        description='Frame ID for marker visualization'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    db_eps_arg = DeclareLaunchArgument(
        'db_eps',
        default_value='0.25',
        description='DBSCAN epsilon parameter (cluster radius in meters)'
    )

    db_min_samples_arg = DeclareLaunchArgument(
        'db_min_samples',
        default_value='5',
        description='DBSCAN minimum samples (minimum points in cluster)'
    )

    roi_min_dist_arg = DeclareLaunchArgument(
        'roi_min_dist',
        default_value='0.20',
        description='ROI minimum distance (meters)'
    )

    roi_max_dist_arg = DeclareLaunchArgument(
        'roi_max_dist',
        default_value='3.00',
        description='ROI maximum distance (meters)'
    )

    center_scale_arg = DeclareLaunchArgument(
        'center_scale',
        default_value='0.12',
        description='Scale of center sphere markers'
    )

    tf_timeout_arg = DeclareLaunchArgument(
        'tf_timeout',
        default_value='0.3',
        description='TF lookup timeout (seconds)'
    )

    # Launch configurations
    scan_topic = LaunchConfiguration('scan_topic')
    marker_frame_id = LaunchConfiguration('marker_frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    db_eps = LaunchConfiguration('db_eps')
    db_min_samples = LaunchConfiguration('db_min_samples')
    roi_min_dist = LaunchConfiguration('roi_min_dist')
    roi_max_dist = LaunchConfiguration('roi_max_dist')
    center_scale = LaunchConfiguration('center_scale')
    tf_timeout = LaunchConfiguration('tf_timeout')

    # Ring visualization node
    ring_viz_node = Node(
        package='obs_detect',
        executable='ring_viz_node',
        name='ring_viz_node',
        output='screen',
        parameters=[{
            'scan_topic': scan_topic,
            'marker_frame_id': marker_frame_id,
            'use_sim_time': use_sim_time,
            'db_eps': db_eps,
            'db_min_samples': db_min_samples,
            'roi_min_dist': roi_min_dist,
            'roi_max_dist': roi_max_dist,
            'center_scale': center_scale,
            'tf_timeout': tf_timeout,
        }],
        remappings=[
            # Uncomment if you need to remap topics
            # ('/scan', '/your_custom_scan_topic'),
        ]
    )

    return LaunchDescription([
        scan_topic_arg,
        marker_frame_arg,
        use_sim_time_arg,
        db_eps_arg,
        db_min_samples_arg,
        roi_min_dist_arg,
        roi_max_dist_arg,
        center_scale_arg,
        tf_timeout_arg,
        ring_viz_node,
    ])
