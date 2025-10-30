from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for boundary_viz_node
    Visualizes track boundaries from CSV files in RViz
    """

    # Launch arguments
    inner_csv_arg = DeclareLaunchArgument(
        'inner_bound_csv',
        default_value='/home/ircv7/RACE/bound/1030/inner_bound_world.csv',
        description='Path to inner boundary CSV file'
    )

    outer_csv_arg = DeclareLaunchArgument(
        'outer_bound_csv',
        default_value='/home/ircv7/RACE/bound/1030/outer_bound_world.csv',
        description='Path to outer boundary CSV file'
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame_id',
        default_value='map',
        description='Frame ID for marker visualization'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Marker publish rate (Hz)'
    )

    line_width_arg = DeclareLaunchArgument(
        'line_width',
        default_value='0.05',
        description='Width of boundary lines'
    )

    z_height_arg = DeclareLaunchArgument(
        'z_height',
        default_value='0.0',
        description='Z height of boundary visualization'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Launch configurations
    inner_bound_csv = LaunchConfiguration('inner_bound_csv')
    outer_bound_csv = LaunchConfiguration('outer_bound_csv')
    marker_frame_id = LaunchConfiguration('marker_frame_id')
    publish_rate = LaunchConfiguration('publish_rate')
    line_width = LaunchConfiguration('line_width')
    z_height = LaunchConfiguration('z_height')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Boundary visualization node
    boundary_viz_node = Node(
        package='obs_detect',
        executable='boundary_viz_node',
        name='boundary_viz_node',
        output='screen',
        parameters=[{
            'inner_bound_csv': inner_bound_csv,
            'outer_bound_csv': outer_bound_csv,
            'marker_frame_id': marker_frame_id,
            'publish_rate': publish_rate,
            'line_width': line_width,
            'z_height': z_height,
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        inner_csv_arg,
        outer_csv_arg,
        marker_frame_arg,
        publish_rate_arg,
        line_width_arg,
        z_height_arg,
        use_sim_time_arg,
        boundary_viz_node,
    ])
