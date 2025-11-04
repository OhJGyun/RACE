#!/usr/bin/env python3

"""
F1TENTH SLAM & Navigation Unified Launch File

This launch file provides a unified interface for both SLAM and localization modes.
It automatically switches between mapping and localization based on the mode parameter.

Usage:
  # SLAM mode (mapping)
  ros2 launch slam_nav slam_nav_launch.py mode:=slam slam_backend:=slam_toolbox
  
  # Localization mode (using existing map)  
  ros2 launch slam_nav slam_nav_launch.py mode:=localization map_yaml_file:=/path/to/map.yaml
  
  # Cartographer SLAM
  ros2 launch slam_nav slam_nav_launch.py mode:=slam slam_backend:=cartographer
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("slam_nav")
    f1tenth_stack_share = get_package_share_directory("f1tenth_stack")

    # Get launch configurations
    mode = LaunchConfiguration("mode").perform(context)
    slam_backend = LaunchConfiguration("slam_backend").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    slam_mode = LaunchConfiguration("slam_mode")
    slam_config_file = LaunchConfiguration("slam_config_file")
    
    map_controller_visualization = LaunchConfiguration("launch_rviz")
    rviz2_config = LaunchConfiguration("rviz2_config")

    launch_actions = []

    if mode == "slam":
        # Include F1TENTH bringup
        bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([f1tenth_stack_share, "launch", "bringup_launch.py"])
            ]),
            launch_arguments={
                "use_sim_time": use_sim_time
            }.items()
        )
        launch_actions.append(bringup_launch)

        # SLAM Toolbox nodes (sync / async)
        slam_toolbox_group = GroupAction([
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_config_file,
                    {"use_sim_time": use_sim_time}
                ],
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration("slam_backend"), "' == 'slam_toolbox' and '", slam_mode, "' == 'sync'"
                    ])
                )
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_config_file,
                    {"use_sim_time": use_sim_time}
                ],
                condition=IfCondition(
                    PythonExpression([
                        "'", LaunchConfiguration("slam_backend"), "' == 'slam_toolbox' and '", slam_mode, "' == 'async'"
                    ])
                )
            )
        ])
        launch_actions.append(slam_toolbox_group)

        # Cartographer nodes
        cartographer_group = GroupAction([
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory", PathJoinSubstitution([pkg_share, "config"]),
                    "-configuration_basename", "cartographer_config.lua"
                ],
                remappings=[
                    ("/scan", "scan"),
                    ("/odom", "odom")
                ],
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("slam_backend"), "' == 'cartographer'"])
                )
            ),
            Node(
                package="cartographer_ros",
                executable="occupancy_grid_node",
                name="occupancy_grid_node",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "resolution": 0.05,
                    "publish_period_sec": 1.0
                }],
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("slam_backend"), "' == 'cartographer'"])
                )
            )
        ])
        launch_actions.append(cartographer_group)

    elif mode == "localization":
        # Localization mode - include localization launch
        localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, "launch", "localization_launch.py"])
            ]),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map_yaml_file": map_yaml_file,
                "autostart": LaunchConfiguration("autostart")
            }.items()
        )
        launch_actions.append(localization_launch)

    return launch_actions


def generate_launch_description():
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="slam",
        choices=["slam", "localization"],
        description="Mode: 'slam' for mapping, 'localization' for using existing map"
    )
    
    slam_backend_arg = DeclareLaunchArgument(
        "slam_backend", 
        default_value="slam_toolbox",
        choices=["slam_toolbox", "cartographer"],
        description="SLAM backend to use (only applies in slam mode)"
    )
    
    slam_mode_arg = DeclareLaunchArgument(
        "slam_mode",
        default_value="sync",
        choices=["sync", "async"],
        description="SLAM mode for slam_toolbox"
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false", 
        description="Use simulation time if true"
    )
    
    map_yaml_file_arg = DeclareLaunchArgument(
        "map_yaml_file",
        default_value="/home/f1/f1tenth_ws/maps/gap_map_final.yaml",
        description="Path to map yaml file (only applies in localization mode)"
    )
    
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically start lifecycle nodes"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config_file",
        default_value=PathJoinSubstitution([get_package_share_directory("slam_nav"), "config", "slam_toolbox_config.yaml"]),
        description="Path to SLAM config file"
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Automatically launch RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz2_config',
        default_value=PathJoinSubstitution([get_package_share_directory("map_control"), "rviz2_config", "/home/ircv7/RACE/real_ws/rviz2_config/slam_nav_visualization.rviz"]),
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
        mode_arg,
        slam_backend_arg,
        slam_mode_arg,
        use_sim_time_arg,
        map_yaml_file_arg,
        autostart_arg,
        slam_config_arg,
        launch_rviz_arg,
        rviz_config_arg,
        rviz_node,
        OpaqueFunction(function=launch_setup)
    ])
