from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('obs_detect')
    config = os.path.join(pkg_share, 'config', 'ring_viz_params.yaml')

    map_controller_visualization = LaunchConfiguration("launch_rviz")
    rviz2_config = LaunchConfiguration("rviz2_config")

    obs_detect_node = Node(
        package='obs_detect',
        executable='obs_detect_node',
        name='obs_detect_node',
        output='screen',
        parameters=[config],
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Automatically launch RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz2_config',
        default_value=PathJoinSubstitution([get_package_share_directory("obs_detect"), "rviz2_config", "obs_detect_visualization.rviz"]),
        description='RViz Config File'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="rviz2",
        arguments=["-d", rviz2_config],
        condition=IfCondition(map_controller_visualization)
    )


    return LaunchDescription([
        obs_detect_node,
        launch_rviz_arg,
        rviz_config_arg,
        rviz_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
