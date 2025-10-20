#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value='/home/f1/f1tenth_ws/maps/gap_map_final.yaml',
        description='Full path to map yaml file to load'
    )
    
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value='/home/f1/f1tenth_ws/config/amcl_simple.yaml',
        description='Full path to the ROS2 parameters file to use for Nav2 AMCL'
    )
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch configurations
    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('params_file')
    sim_time = LaunchConfiguration('use_sim_time')

    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': sim_time,
            'yaml_filename': map_yaml
        }]
    )

    # Nav2 AMCL node (Particle Filter based)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': sim_time}
        ]
    )

    # Lifecycle manager for localization nodes
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    return LaunchDescription([
        # Declare launch arguments
        map_yaml_file,
        params_file,
        use_sim_time,

        # Launch nodes
        map_server_node,
        amcl_node,
        lifecycle_manager_localization
    ])
