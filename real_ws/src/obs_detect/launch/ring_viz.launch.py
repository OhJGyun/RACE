#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # 파라미터 파일 경로를 런치 인자로 받도록
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            os.getenv("HOME", "/home/ircv7/"),
            "RACE/real_ws/src/obs_detect/config/ring_viz_params.yaml"
        ),
        description="YAML parameter file for ring_viz_node",
    )

    params_file = LaunchConfiguration("params_file")

    scan_viz_node = Node(
        package="obs_detect",          # <- 네 패키지 이름
        executable="ring_viz_node",     # <- setup.py console_scripts에 등록된 이름
        name="ring_viz_node",
        output="screen",
        parameters=[params_file],
        remappings=[
            # LaserScan 토픽 이름 바꾸고 싶으면 여기서 remap 가능
            # ("scan_viz/markers", "/scan_viz/markers"),  # 그대로 쓰면 없어도 됨
        ],
    )

    return LaunchDescription([
        params_file_arg,
        scan_viz_node,
    ])
