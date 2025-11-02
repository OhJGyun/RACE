#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    ring_viz_node (SimpleScanViz) 를 파라미터 YAML과 함께 띄우는 launch 파일.

    전제:
    - 패키지 이름: obs_detect
    - setup.py 내 console_scripts:
        "ring_viz_node = obs_detect.simple_scan_viz:main"
      이런 식으로 등록돼 있다고 가정.
      (즉 ros2 run obs_detect ring_viz_node 가 돌아간다고 가정)
    - config/ring_viz.yaml 안에 우리가 선언했던 모든 declare_parameter 값이 들어있음.
    """

    # 패키지 내부 구조 가정:
    # your_pkg/
    #   launch/ring_viz.launch.py   (요 파일)
    #   config/ring_viz.yaml        (파라미터)
    #
    # launch/에서 ../config/ring_viz.yaml 상대 경로로 접근
    this_dir = os.path.dirname(__file__)
    config_path = os.path.normpath(
        os.path.join(this_dir, "..", "config", "ring_viz.yaml")
    )

    ring_viz = Node(
        package="obs_detect",          # ← 실제 패키지명으로 맞춰줘
        executable="ring_viz_node",    # ← setup.py console_scripts 이름
        name="ring_viz_node",          # ← rclpy에서 보일 node name
        output="screen",
        parameters=[config_path],
        remappings=[
            # (= topic remap)
            # 기본 코드에서는:
            #   LaserScan 구독: /scan   (scan_topic 파라미터로도 바꿀 수 있음)
            #   MarkerArray 퍼블리시: scan_viz/markers
            #   PoseArray 퍼블리시:   scan_viz/obstacles
            #
            # 필요한 경우 주석 풀어서 사용:
            # ("scan_viz/obstacles", "/lane_selector/obstacles_in_ring"),
            # ("/scan", "/scan_front"),
        ],
    )

    return LaunchDescription([ring_viz])
