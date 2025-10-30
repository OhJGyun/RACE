#!/usr/bin/env python3
"""
Steering Test Launch File

다양한 테스트 시나리오:
1. Step 응답 테스트 (급격한 변화)
2. Sine 파형 테스트 (주파수 응답)
3. Square 파형 테스트 (지연 측정)
4. Triangle 파형 테스트 (선형 변화)

Usage:
    # Step 입력 테스트 (0.5Hz)
    ros2 launch app steering_test.launch.py waveform:=step frequency:=0.5

    # Sine 파형 테스트 (1Hz)
    ros2 launch app steering_test.launch.py waveform:=sine frequency:=1.0

    # High frequency sine test
    ros2 launch app steering_test.launch.py waveform:=sine frequency:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    waveform_arg = DeclareLaunchArgument(
        'waveform',
        default_value='step',
        description='Waveform type: step, sine, square, triangle, chirp'
    )

    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='0.5',
        description='Waveform frequency in Hz'
    )

    amplitude_arg = DeclareLaunchArgument(
        'amplitude',
        default_value='0.3',
        description='Steering amplitude in radians (±17 degrees default)'
    )

    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.0',
        description='Constant speed in m/s'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='40.0',
        description='Publish rate in Hz'
    )

    test_duration_arg = DeclareLaunchArgument(
        'test_duration',
        default_value='30.0',
        description='Test duration in seconds (0 = infinite)'
    )

    # Steering test node
    steering_test_node = Node(
        package='pure_pursuit',
        executable='steering_test_node.py',
        name='steering_test_node',
        output='screen',
        parameters=[{
            'waveform': LaunchConfiguration('waveform'),
            'frequency': LaunchConfiguration('frequency'),
            'amplitude': LaunchConfiguration('amplitude'),
            'speed': LaunchConfiguration('speed'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'test_duration': LaunchConfiguration('test_duration'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        waveform_arg,
        frequency_arg,
        amplitude_arg,
        speed_arg,
        publish_rate_arg,
        test_duration_arg,
        steering_test_node,
    ])
