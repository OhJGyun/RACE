from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for ring_viz_node (SimpleScanViz)
    Uses fast scanline clustering with parameters eps0, k
    """

    # -------------------------------
    # Declare arguments
    # -------------------------------
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='LaserScan topic to subscribe to'
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame_id', default_value='map',
        description='Frame ID for marker visualization'
    )

    tf_timeout_arg = DeclareLaunchArgument(
        'tf_timeout', default_value='0.3',
        description='TF lookup timeout (seconds)'
    )

    db_min_samples_arg = DeclareLaunchArgument(
        'db_min_samples', default_value='8',
        description='Minimum number of points to form a cluster'
    )

    roi_min_dist_arg = DeclareLaunchArgument(
        'roi_min_dist', default_value='0.20',
        description='Minimum ROI distance (m)'
    )

    roi_max_dist_arg = DeclareLaunchArgument(
        'roi_max_dist', default_value='6.00',
        description='Maximum ROI distance (m)'
    )

    center_scale_arg = DeclareLaunchArgument(
        'center_scale', default_value='0.3',
        description='Sphere marker scale for cluster centers'
    )

    fov_deg_arg = DeclareLaunchArgument(
        'fov_deg', default_value='120.0',
        description='Field of view in degrees'
    )

    fov_center_deg_arg = DeclareLaunchArgument(
        'fov_center_deg', default_value='0.0',
        description='FOV center offset in degrees'
    )

    eps0_arg = DeclareLaunchArgument(
        'eps0', default_value='0.12',
        description='Base distance threshold (m) for scanline clustering'
    )

    k_arg = DeclareLaunchArgument(
        'k', default_value='0.06',
        description='Distance ratio coefficient for adaptive linking'
    )

    outer_bound_csv_arg = DeclareLaunchArgument(
        'outer_bound_csv',
        default_value='/home/ircv7/RACE/bound/1031_1/outer_bound.csv',
        description='Outer boundary CSV path'
    )

    inner_bound_csv_arg = DeclareLaunchArgument(
        'inner_bound_csv',
        default_value='/home/ircv7/RACE/bound/1031_1/inner_bound.csv',
        description='Inner boundary CSV path'
    )

    # -------------------------------
    # Configurations
    # -------------------------------
    scan_topic = LaunchConfiguration('scan_topic')
    marker_frame_id = LaunchConfiguration('marker_frame_id')
    tf_timeout = LaunchConfiguration('tf_timeout')
    db_min_samples = LaunchConfiguration('db_min_samples')
    roi_min_dist = LaunchConfiguration('roi_min_dist')
    roi_max_dist = LaunchConfiguration('roi_max_dist')
    center_scale = LaunchConfiguration('center_scale')
    fov_deg = LaunchConfiguration('fov_deg')
    fov_center_deg = LaunchConfiguration('fov_center_deg')
    eps0 = LaunchConfiguration('eps0')
    k = LaunchConfiguration('k')
    outer_bound_csv = LaunchConfiguration('outer_bound_csv')
    inner_bound_csv = LaunchConfiguration('inner_bound_csv')

    # -------------------------------
    # Node definition
    # -------------------------------
    ring_viz_node = Node(
        package='obs_detect',
        executable='ring_viz_node',
        name='ring_viz_node',
        output='screen',
        parameters=[{
            'scan_topic': scan_topic,
            'marker_frame_id': marker_frame_id,
            'tf_timeout': tf_timeout,
            'db_min_samples': db_min_samples,
            'roi_min_dist': roi_min_dist,
            'roi_max_dist': roi_max_dist,
            'center_scale': center_scale,
            'fov_deg': fov_deg,
            'fov_center_deg': fov_center_deg,
            'eps0': eps0,
            'k': k,
            'outer_bound_csv': outer_bound_csv,
            'inner_bound_csv': inner_bound_csv,
        }],
    )

    # -------------------------------
    # LaunchDescription
    # -------------------------------
    return LaunchDescription([
        scan_topic_arg,
        marker_frame_arg,
        tf_timeout_arg,
        db_min_samples_arg,
        roi_min_dist_arg,
        roi_max_dist_arg,
        center_scale_arg,
        fov_deg_arg,
        fov_center_deg_arg,
        eps0_arg,
        k_arg,
        outer_bound_csv_arg,
        inner_bound_csv_arg,
        ring_viz_node,
    ])