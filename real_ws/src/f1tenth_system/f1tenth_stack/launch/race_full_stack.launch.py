#!/usr/bin/env python3
"""
F1TENTH Race Full Stack Launch File

통합 실행 시스템 - 5개의 독립적인 런치 파일을 한번에 실행:
1. ros2 launch slam_nav localization_launch.py
2. ros2 launch map_control map_controller.launch.py
3. ros2 launch lane_selector lane_selector_launch.py
4. ros2 launch obs_detect ring_viz.launch.py
5. ros2 launch obs_detect boundary_viz.launch.py

Usage:
    ros2 launch f1tenth_stack race_full_stack.launch.py
    ros2 launch f1tenth_stack race_full_stack.launch.py use_sim_time:=false
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate integrated launch description for full F1TENTH race stack
    """

    # Package directories
    slam_nav_pkg = get_package_share_directory('slam_nav')
    map_control_pkg = get_package_share_directory('map_control')
    lane_selector_pkg = get_package_share_directory('lane_selector')
    obs_detect_pkg = get_package_share_directory('obs_detect')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ========================================================================
    # 1. SLAM & Localization
    #    ros2 launch slam_nav localization_launch.py
    # ========================================================================
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([slam_nav_pkg, 'launch', 'localization_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ========================================================================
    # 2. MAP Controller (Path Following)
    #    ros2 launch map_control map_controller.launch.py
    # ========================================================================
    map_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([map_control_pkg, 'launch', 'map_controller.launch.py'])
        ]),
        launch_arguments={
            'launch_rviz': 'false',  # RViz는 localization에서 이미 실행됨
        }.items()
    )

    # ========================================================================
    # 3. Lane Selector (Dynamic Lane Switching)
    #    ros2 launch lane_selector lane_selector_launch.py
    # ========================================================================
    lane_selector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([lane_selector_pkg, 'launch', 'lane_selector_launch.py'])
        ])
    )

    # ========================================================================
    # 4. Obstacle Detection - Ring Clustering
    #    ros2 launch obs_detect ring_viz.launch.py
    # ========================================================================
    ring_viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([obs_detect_pkg, 'launch', 'ring_viz.launch.py'])
        ])
    )

    # ========================================================================
    # 5. Obstacle Detection - Boundary Visualization
    #    ros2 launch obs_detect boundary_viz.launch.py
    # ========================================================================
    boundary_viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([obs_detect_pkg, 'launch', 'boundary_viz.launch.py'])
        ]),
        launch_arguments={
            'launch_rviz': 'false',  # RViz는 localization에서 이미 실행됨
        }.items()
    )

    # ========================================================================
    # Launch Description
    # ========================================================================
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,

        # Launch files (5개의 독립적인 런치 파일 실행)
        slam_localization,        # 1. SLAM & Localization (includes RViz)
        map_controller_launch,    # 2. MAP Controller
        lane_selector_launch,     # 3. Lane Selector
        ring_viz_launch,          # 4. Ring Obstacle Detection
        boundary_viz_launch,      # 5. Boundary Visualization
    ])


if __name__ == '__main__':
    generate_launch_description()
