#!/usr/bin/env python3
"""
Launch file for disparity-based autonomous driving
Launches disparity autodrive node with parameters
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('disparity_autodrive')

    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'disparity_params.yaml')

    # Disparity autodrive node
    disparity_node = Node(
        package='disparity_autodrive',
        executable='disparity_autodrive_node',
        name='disparity_autodrive_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        disparity_node,
    ])
