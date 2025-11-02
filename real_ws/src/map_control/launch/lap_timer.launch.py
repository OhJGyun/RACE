#!/usr/bin/env python3
"""
Launch file for Lap Timer Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_control',
            executable='lap_timer_node',
            name='lap_timer_node',
            output='screen',
            parameters=[{
                'tf_timeout': 0.3,
                'marker_frame_id': 'map',
                'update_rate_hz': 50.0,
            }]
        )
    ])
