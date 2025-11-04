from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch MAP Controller OJG TEST (FORZA-restored features) with default configuration

    Features:
    - TF-based localization (map -> base_link) for real car
    - FORZA-restored features: curvature-based speed limiting, heading adjustment
    - Full CSV format support (x, y, v, d, s, kappa, psi, ax)
    - Multi-lane support
    - Lane selector integration
    - Local waypoint segment optimization
    - Collision recovery system
    - Path and lookahead visualization
    """
    pkg_share = get_package_share_directory('map_control')
    config = os.path.join(pkg_share, 'config', 'map_control_params.yaml')

    map_controller_visualization = LaunchConfiguration("launch_rviz")
    rviz2_config = LaunchConfiguration("rviz2_config")

    # 🔧 MAP Controller OJG Test Node
    map_controller_node = Node(
        package='map_control',
        executable='map_controller_ojg_test',  # Using OJG test version with FORZA features
        name='map_controller',
        output='screen',
        parameters=[config],
    )

    # Drive relay (mux input)
    drive_relay_node = Node(
        package='map_control',
        executable='drive_relay',
        name='drive_relay',
        output='screen',
    )

    # Collision recovery
    collision_recovery_node = Node(
        package='map_control',
        executable='collision_recovery_node',
        name='collision_recovery_node',
        output='screen',
        parameters=[{
            'stuck_distance_threshold': 1.5,
            'stuck_time_threshold': 2.0,
            'reverse_speed': -0.5,
            'reverse_duration': 1.0,
            'map_frame': 'map',
            'base_frame': 'base_link',
            'command_topic': '/recovery/ackermann_cmd',
        }],
    )

    # Ackermann command multiplexer
    ackermann_cmd_mux_node = Node(
        package='map_control',
        executable='ackermann_cmd_mux',
        name='ackermann_cmd_mux',
        output='screen',
        parameters=[{
            'primary_topic': '/map_control/ackermann_cmd',
            'recovery_topic': '/recovery/ackermann_cmd',
            'output_topic': '/ackermann_cmd',
            'recovery_hold_time': 0.3,
        }],
    )

    # Visualization nodes
    steer_viz_node = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='steer_viz_node',
        output='screen',
        parameters=[
            {"string_topic": "/viz/steer"},
            {"fg_color": "w"},
        ],
    )

    speed_viz_node = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='speed_viz_node',
        output='screen',
        parameters=[
            {"string_topic": "/viz/speed"},
            {"fg_color": "w"},
        ],
    )

    lap_time_viz_node = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='lap_time_viz_node',
        output='screen',
        parameters=[
            {"string_topic": "/viz/lap_time"},
            {"fg_color": "w"},
        ],
    )

    obs_num_viz_node = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='obs_num_viz_node',
        output='screen',
        parameters=[
            {"string_topic": "/viz/obs_num"},
            {"fg_color": "w"},
        ],
    )

    mode_viz_node = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='mode_viz_node',
        output='screen',
        parameters=[
            {"string_topic": "/viz/mode"},
            {"fg_color": "w"},
        ],
    )

    lane_num_viz_node = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='lane_num_viz_node',
        output='screen',
        parameters=[
            {"string_topic": "/viz/lane_num"},
            {"fg_color": "w"},
        ],
    )

    # Launch arguments
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Automatically launch RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz2_config',
        default_value='/home/ircv7/RACE/real_ws/rviz2_config/slam_nav_visualization_joon.rviz',
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
        collision_recovery_node,
        ackermann_cmd_mux_node,
        steer_viz_node,
        speed_viz_node,
        lap_time_viz_node,
        obs_num_viz_node,
        mode_viz_node,
        lane_num_viz_node,
        launch_rviz_arg,
        rviz_config_arg,
        rviz_node
    ])


if __name__ == '__main__':
    generate_launch_description()
