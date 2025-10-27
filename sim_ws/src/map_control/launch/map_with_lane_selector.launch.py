from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch MAP Controller with Lane Selector integration

    Launches:
    - map_controller (MAP Humble)
    - lane_selector (obstacle-based lane selection)

    Note: Ensure lane_csv_paths in map_controller matches lane_selector config!
    """
    map_pkg_share = get_package_share_directory('map_control')
    map_config = os.path.join(map_pkg_share, 'config', 'map_controller_params.yaml')

    # Try to get lane_selector config (if package exists)
    try:
        lane_pkg_share = get_package_share_directory('lane_selector')
        lane_config = os.path.join(lane_pkg_share, 'config', 'lane_selector.yaml')
        has_lane_selector = os.path.exists(lane_config)
    except:
        has_lane_selector = False
        lane_config = None

    map_controller_node = Node(
        package='map_control',
        executable='map_control',
        name='map_controller',
        output='screen',
        parameters=[map_config],
    )

    nodes = [map_controller_node]

    if has_lane_selector:
        lane_selector_node = Node(
            package='lane_selector',
            executable='lane_selector_node',
            name='lane_selector_node',
            output='screen',
            parameters=[lane_config],
        )
        nodes.append(lane_selector_node)

    return LaunchDescription(nodes)


if __name__ == '__main__':
    generate_launch_description()
