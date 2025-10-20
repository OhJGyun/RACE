from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lane_selector')
    config = os.path.join(pkg_share, 'config', 'lane_selector.yaml')

    lane_selector_node = Node(
        package='lane_selector',
        executable='lane_selector_node',
        name='lane_selector_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([lane_selector_node])


if __name__ == '__main__':
    generate_launch_description()
