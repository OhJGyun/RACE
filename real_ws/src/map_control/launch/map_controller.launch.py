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
    - Collision recovery system
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

    collision_recovery_node = Node(
            package='map_control',
            executable='collision_recovery_node',
            name='collision_recovery_node',
            output='screen',
            parameters=[{
                'enable_recovery': True,               # 🔧 Enable/disable collision recovery (True/False)
                'imu_accel_min': -1.0,                 # Min linear_x accel for collision [m/s²]
                'imu_accel_max': 0.0,                  # Max linear_x accel for collision [m/s²]
                'collision_time_threshold': 0.5,       # Time to confirm collision [s]
                'commanded_speed_threshold': 0.3,      # Min commanded speed to monitor [m/s]
                'reverse_speed': -1.5,                 # Reverse speed [m/s]
                'reverse_duration': 1.0,               # Reverse duration [s]
                'imu_topic': '/sensors/imu/raw',       # f1tenth publishes to /sensors/imu/raw
                'command_topic': '/recovery/ackermann_cmd',
                'drive_cmd_topic': '/map_control/ackermann_cmd',
                'base_frame': 'base_link',
            }],
    )

    ackermann_cmd_mux_node = Node(
            package='map_control',
            executable='ackermann_cmd_mux',
            name='ackermann_cmd_mux',
            output='screen',
            parameters=[{
                'joy_topic': '/joy_teleop/ackermann_cmd',      # Priority 1: Manual control (highest)
                'recovery_topic': '/recovery/ackermann_cmd',   # Priority 2: Collision recovery
                'primary_topic': '/map_control/ackermann_cmd', # Priority 3: Autonomous control
                'output_topic': '/drive',                      # f1tenth_system uses /drive
                'joy_timeout': 0.5,                            # Joy active if cmd within 0.5s
                'recovery_hold_time': 0.3,                     # Recovery duration
            }],
    )

    steer_viz_node = Node(
            package='rviz_2d_overlay_plugins',
            executable='string_to_overlay_text',
            name='steer_viz_node',
            output='screen',
            parameters=[
                {"string_topic": "/viz/steer"},
                {"fg_color": "w"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            ],
    )

    speed_viz_node = Node(
            package='rviz_2d_overlay_plugins',
            executable='string_to_overlay_text',
            name='speed_viz_node',
            output='screen',
            parameters=[
                {"string_topic": "/viz/speed"},
                {"fg_color": "w"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            ],
    )

    lap_time_viz_node = Node(
            package='rviz_2d_overlay_plugins',
            executable='string_to_overlay_text',
            name='lap_time_viz_node',
            output='screen',
            parameters=[
                {"string_topic": "/viz/lap_time"},
                {"fg_color": "w"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            ],
    )

    obs_num_viz_node = Node(
            package='rviz_2d_overlay_plugins',
            executable='string_to_overlay_text',
            name='obs_num_viz_node',
            output='screen',
            parameters=[
                {"string_topic": "/viz/obs_num"},
                {"fg_color": "w"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            ],
    )

    mode_viz_node = Node(
            package='rviz_2d_overlay_plugins',
            executable='string_to_overlay_text',
            name='mode_viz_node',
            output='screen',
            parameters=[
                {"string_topic": "/viz/mode"},
                {"fg_color": "w"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            ],
    )

    lane_num_viz_node = Node(
            package='rviz_2d_overlay_plugins',
            executable='string_to_overlay_text',
            name='lane_num_viz_node',
            output='screen',
            parameters=[
                {"string_topic": "/viz/lane_num"},
                {"fg_color": "w"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            ],
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Automatically launch RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz2_config',
        default_value='/home/ircv7/RACE/real_ws/rviz2_config/slam_nav_visualization_joon.rviz',
        description='RViz Config File (slam_nav_visualization_joon.rviz)'
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
