from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time        = LaunchConfiguration('use_sim_time', default='false')
    resolution          = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec  = LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument('resolution', default_value='0.05',
                              description='Grid resolution (m/px)'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0',
                              description='OccupancyGrid publish period (s)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),

        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }]
        ),
    ])
