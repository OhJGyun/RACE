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

    obs_detect_node = Node(
        package='obs_detect',
        executable='obs_detect_node',
        name='obs_detect_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        obs_detect_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
