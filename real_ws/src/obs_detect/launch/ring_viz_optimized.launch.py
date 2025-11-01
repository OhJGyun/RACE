#!/usr/bin/env python3
"""
Launch file for optimized ring_viz_node with config file
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('obs_detect')

    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'obs_detect_params.yaml')

    return LaunchDescription([
        Node(
            package='obs_detect',
            executable='ring_viz_optimized',
            name='ring_viz_optimized',
            output='screen',
            parameters=[config_file],
            remappings=[
                # Add remappings if needed
            ]
        )
    ])
