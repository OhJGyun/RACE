from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('obs_detect')
    config = os.path.join(pkg_share, 'config', 'obs_detect.yaml')

    obs_detect_node = Node(
        package='obs_detect',
        executable='obs_detect_node',
        name='obs_detect_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([obs_detect_node])


if __name__ == '__main__':
    generate_launch_description()
