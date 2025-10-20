import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    # f1tenth_slam_params.yaml 파일 사용
    default_params_file = os.path.join('/home/f1/f1tenth_ws/config', 'f1tenth_slam_params.yaml')
    if not os.path.exists(default_params_file):
        print(f"Warning: f1tenth_slam_params.yaml not found at {default_params_file}")
        # slam_toolbox 기본 설정으로 폴백
        try:
            slam_toolbox_share = get_package_share_directory("slam_toolbox")
            default_params_file = os.path.join(slam_toolbox_share, 'config', 'mapper_params_online_sync.yaml')
        except Exception:
            print("Warning: slam_toolbox package not found. Using empty parameters.")
            default_params_file = ""

    # URDF 파일 경로 (패키지 내 urdf 디렉토리)
    try:
        package_share_directory = get_package_share_directory('f1tenth_stack')
        urdf_file = os.path.join(package_share_directory, 'urdf', 'f1tenth_car.urdf')
    except Exception:
        # Fallback to source directory if package not installed
        urdf_file = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'f1tenth_car.urdf')
        urdf_file = os.path.abspath(urdf_file)
    
    # URDF 파일 읽기 (에러 처리 포함)
    try:
        with open(urdf_file, 'r') as infp:
            robot_desc = infp.read()
    except FileNotFoundError:
        print(f"Error: URDF file not found at {urdf_file}")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading URDF file: {e}")
        sys.exit(1)

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # slam_toolbox 매개변수 처리 (패키지가 있는 경우에만)
    if default_params_file:
        # If the provided param file doesn't have slam_toolbox params, we must pass the
        # default_params_file instead. This could happen due to automatic propagation of
        # LaunchArguments. See:
        # https://github.com/ros-planning/navigation2/pull/2243#issuecomment-800479866
        has_node_params = HasNodeParams(source_file=params_file,
                                        node_name='slam_toolbox')

        actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                               ' else "', default_params_file, '"'])

        log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                        ' does not contain slam_toolbox parameters. Using default: ',
                                        default_params_file],
                                   condition=UnlessCondition(has_node_params))
    else:
        actual_params_file = params_file
        log_param_change = LogInfo(msg=['slam_toolbox package not found, using provided params_file: ', params_file])

    # Robot State Publisher - 차량 모델 표시
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc
        }]
    )

    # Static Transform for map->odom (임시)
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Joint State Publisher - 고정 조인트용
    start_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # slam_toolbox 노드 (패키지가 있는 경우에만 실행)
    nodes_to_add = []
    if default_params_file:
        start_sync_slam_toolbox_node = Node(
            parameters=[
              actual_params_file,
              {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen')
        nodes_to_add.append(start_sync_slam_toolbox_node)
    else:
        print("slam_toolbox package not available, skipping SLAM node.")

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_joint_state_publisher)
    
    # 임시로 SLAM과 static transform 제거하여 디버깅
    # ld.add_action(declare_params_file_cmd)
    # ld.add_action(log_param_change)
    # ld.add_action(static_transform_publisher)
    # for node in nodes_to_add:
    #     ld.add_action(node)

    return ld
