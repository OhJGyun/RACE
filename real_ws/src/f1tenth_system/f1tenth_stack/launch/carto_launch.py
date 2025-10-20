from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("f1tenth_stack")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    config_basename_arg = DeclareLaunchArgument("config_basename", default_value="f110_2d.lua")
    config_dir_arg = DeclareLaunchArgument("config_dir", default_value=os.path.join(pkg_share, "config"))

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "bringup_launch.py"))
    )

    # ① 브릿지 노드: /sensors/imu (VescImuStamped) → /imu/vesc (sensor_msgs/Imu)
    vesc_imu_bridge = Node(
        package="f1tenth_stack",
        executable="vesc_imu_bridge",     # setup.py console_scripts와 동일
        name="vesc_imu_bridge",
        output="screen",
        parameters=[{"frame_id": "base_link"},
        {"roll_sign": -1.0},   # roll 반전
        {"pitch_sign": 1.0},
        {"yaw_sign": 1.0},],  # 필요시 프레임명 조정
    )

    # ② Cartographer: 브릿지 출력만 구독
    carto_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "-configuration_directory", LaunchConfiguration("config_dir"),
            "-configuration_basename", LaunchConfiguration("config_basename"),
        ],
        remappings=[
            ("/scan", "scan"),
            ("/imu",  "/imu/vesc"),    # ← 브릿지 출력으로 고정
        ],
    )

    occ_grid_node = Node(
        package="cartographer_ros",
        executable="occupancy_grid_node",
        name="occupancy_grid_node",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "resolution": 0.05,
            "publish_period_sec": 1.0,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_basename_arg,
        config_dir_arg,
        bringup_launch,
        vesc_imu_bridge,   # ← cartographer 이전에 올라오게 배치
        carto_node,
        occ_grid_node,
    ])
