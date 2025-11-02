import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file for boundary_viz_node.
    Loads parameters from obs_detect_params.yaml so boundary CSV paths and
    visualization settings can be managed in one place.
    """

    package_share = get_package_share_directory('obs_detect')
    default_params = os.path.join(package_share, 'config', 'obs_detect_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML parameter file for boundary_viz_node'
    )

    params_file = LaunchConfiguration('params_file')

    boundary_viz_node = Node(
        package='obs_detect',
        executable='boundary_viz_node',
        name='boundary_viz_node',
        output='screen',
        parameters=[ParameterFile(params_file, allow_substs=True)],
    )

    return LaunchDescription([
        params_file_arg,
        boundary_viz_node,
    ])
