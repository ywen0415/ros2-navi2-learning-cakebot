#!/usr/bin/env python3

"""Launch the cakebot static-map localization system."""

# 本启动文件只负责静态地图定位：map_server 加载 map 参数指定的地图，AMCL 根据
# /scan 和现有 TF 估计位置，lifecycle_manager 自动激活两个生命周期节点，
# amcl_path_publisher 将 /amcl_pose 转为 /amcl_path；可选启动定位 RViz。
# Gazebo、/scan、/odom 和 odom -> base_link -> laser_link 必须由
# cakebot_description 的 gazebo.launch.py 或真机驱动在本启动文件之前提供。
# 不要与 slam.launch.py 同时运行，因为 SLAM 和 AMCL 都会发布 map -> odom。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
            map_server,
            amcl,
            lifecycle_manager,
            path_publisher,
            rviz,
        ]
    )
