#!/usr/bin/env python3

"""Launch the cakebot static-map localization system."""

# 本启动文件只负责静态地图定位：map_server 加载 map 参数指定的地图，AMCL 根据
# /scan 和现有 TF 估计位置，lifecycle_manager 自动激活两个生命周期节点，
# amcl_path_publisher 将 /amcl_pose 转为 /amcl_path，并可在正常退出时保存
# 完整的 AMCL 估计轨迹 CSV；可选启动定位 RViz。
# Gazebo、/scan、/odom 和 odom -> base_link -> laser_link 必须由
# cakebot_description 的 gazebo.launch.py 或真机驱动在本启动文件之前提供。
# 不要与 slam.launch.py 同时运行，因为 SLAM 和 AMCL 都会发布 map -> odom。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("cakebot_navigation")

    default_params_file = PathJoinSubstitution(
        [package_share, "config", "amcl.yaml"]
    )
    default_rviz_config_file = PathJoinSubstitution(
        [package_share, "rviz", "localization.rviz"]
    )

    map_file = DeclareLaunchArgument(
        "map",
        description=(
            "Nav2 地图 YAML 文件路径，例如 "
            "/home/wen/cakebot/maps/test_env.yaml。"
        ),
    )
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="AMCL 参数文件路径。",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="地图服务器、AMCL 和 RViz 是否使用 Gazebo 仿真时间。",
    )
    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启动定位 RViz。",
    )
    rviz_config_file = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=default_rviz_config_file,
        description="定位 RViz 配置文件路径。",
    )
    save_trajectory = DeclareLaunchArgument(
        "save_trajectory",
        default_value="false",
        description="定位进程正常退出时是否将完整 AMCL 轨迹保存为 CSV。",
    )
    trajectory_file = DeclareLaunchArgument(
        "trajectory_file",
        default_value="amcl_trajectory.csv",
        description="轨迹 CSV 输出路径；相对路径以启动命令的工作目录为基准。",
    )
    trajectory_overwrite = DeclareLaunchArgument(
        "trajectory_overwrite",
        default_value="false",
        description="是否允许覆盖已存在的轨迹 CSV。",
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": LaunchConfiguration("map"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[("scan", "/scan")],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }
        ],
    )

    path_publisher = Node(
        package="cakebot_navigation",
        executable="amcl_path_publisher",
        name="amcl_path_publisher",
        output="screen",
        parameters=[
            {
                "input_topic": "/amcl_pose",
                "output_topic": "/amcl_path",
                "save_csv": ParameterValue(
                    LaunchConfiguration("save_trajectory"), value_type=bool
                ),
                "csv_output_path": ParameterValue(
                    LaunchConfiguration("trajectory_file"), value_type=str
                ),
                "csv_overwrite": ParameterValue(
                    LaunchConfiguration("trajectory_overwrite"), value_type=bool
                ),
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_localization",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            map_file,
            params_file,
            use_sim_time,
            use_rviz,
            rviz_config_file,
            save_trajectory,
            trajectory_file,
            trajectory_overwrite,
            map_server,
            amcl,
            lifecycle_manager,
            path_publisher,
            rviz,
        ]
    )
