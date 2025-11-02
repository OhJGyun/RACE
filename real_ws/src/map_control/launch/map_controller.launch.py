from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch MAP Controller (Unified) with default configuration

    Features:
    - TF-based localization (map -> base_link)
    - Multi-lane support
    - Lane selector integration
    """
    pkg_share = get_package_share_directory('map_control')
    config = os.path.join(pkg_share, 'config', 'map_control_params.yaml')

    map_controller_visualization = LaunchConfiguration("launch_rviz")
    rviz2_config = LaunchConfiguration("rviz2_config")

    map_controller_node = Node(
        package='map_control',
        executable='map_controller',
        name='map_controller',
        output='screen',
        parameters=[config],
    )

    drive_relay_node = Node(
            package='map_control',
            executable='drive_relay',
            name='drive_relay',
            output='screen',
    )

    steer_viz_node = Node(
            package='rviz_2d_overlay_plugins',
            executable='string_to_overlay_text',
            name='string_to_overlay_text',
            output='screen',
            parameters=[
                {"string_topic": "/viz/steering_angle"},
                {"fg_color": "b"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            ],
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Automatically launch RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz2_config',
        default_value=PathJoinSubstitution([get_package_share_directory("map_control"), "rviz2_config", "/home/ircv7/RACE/real_ws/rviz2_config/slam_nav_visualization_joon.rviz"]),
        description='RViz Config File'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz2_config],
        condition=IfCondition(map_controller_visualization)
    )

    return LaunchDescription([
        map_controller_node,
        drive_relay_node,
        steer_viz_node,
        launch_rviz_arg,
        rviz_config_arg,
        rviz_node
    ])


if __name__ == '__main__':
    generate_launch_description()
