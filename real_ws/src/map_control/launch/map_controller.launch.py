from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


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

    map_controller_node = Node(
        package='map_control',
        executable='map_controller',
        name='map_controller',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([map_controller_node])


if __name__ == '__main__':
    generate_launch_description()
