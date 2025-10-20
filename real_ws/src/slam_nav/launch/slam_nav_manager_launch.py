#!/usr/bin/env python3

"""
F1TENTH C++ SLAM/Nav Manager Launch File
C++ 버전의 SLAM/Navigation 매니저를 실행합니다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="시뮬레이션 시간 사용 여부"
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")

    # C++ SLAM/Nav Manager 노드
    slam_nav_manager_node = Node(
        package="slam_nav",
        executable="slam_nav_manager",
        name="slam_nav_manager",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_nav_manager_node
    ])