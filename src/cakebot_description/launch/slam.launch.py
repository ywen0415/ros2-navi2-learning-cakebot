#!/usr/bin/env python3

"""启动 SLAM Toolbox 在线建图和用于建图的 RViz。

用途：订阅已经运行中的 /scan 和 TF，发布 /map 以及 map -> odom，
并可启动配置好的建图 RViz。可选的自动保存节点会缓存 /map，并在本 launch
收到 Ctrl+C 时保存地图。本脚本不启动 Gazebo、不生成机器人实体，因此需要先
启动 gazebo.launch.py；同时使用时应关闭 Gazebo 脚本里的 RViz。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("cakebot_description")

    default_params_file = PathJoinSubstitution(
        [package_share, "config", "slam_toolbox.yaml"]
    )
    default_rviz_config_file = PathJoinSubstitution(
        [package_share, "rviz", "cakebot.rviz"]
    )

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="SLAM Toolbox 和 RViz 是否使用 Gazebo 仿真时间。",
    )
    slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=default_params_file,
        description="SLAM Toolbox 参数文件路径。",
    )
    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否同时启动用于建图的 RViz。",
    )
    rviz_config_file = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=default_rviz_config_file,
        description="RViz 配置文件路径。",
    )
    autosave_on_shutdown = DeclareLaunchArgument(
        "autosave_on_shutdown",
        default_value="false",
        description="收到 Ctrl+C 时是否自动保存最新 /map。",
    )
    autosave_map_file = DeclareLaunchArgument(
        "autosave_map_file",
        default_value="maps/slam_map",
        description="自动保存地图的文件前缀，不含 .yaml 或 .pgm 后缀。",
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_slam",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    map_autosaver = Node(
        package="cakebot_description",
        executable="slam_map_autosaver.py",
        name="slam_map_autosaver",
        output="screen",
        condition=IfCondition(LaunchConfiguration("autosave_on_shutdown")),
        parameters=[
            {
                "output_file": LaunchConfiguration("autosave_map_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            use_sim_time,
            slam_params_file,
            use_rviz,
            rviz_config_file,
            autosave_on_shutdown,
            autosave_map_file,
            slam_toolbox,
            rviz,
            map_autosaver,
        ]
    )
