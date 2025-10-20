#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Car visualizer node
    car_visualizer = Node(
        package='f1tenth_stack',
        executable='car_visualizer.py',
        name='car_visualizer',
        output='screen'
    )

    return LaunchDescription([
        car_visualizer
    ])