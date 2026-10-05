#!/usr/bin/env python3

"""启动 Gazebo Classic 仿真并生成 cakebot。

用途：启动 Gazebo 世界、robot_state_publisher、机器人实体和可选的 RViz。
本脚本不启动 SLAM；建图时通常与 slam.launch.py 配合使用，并将本脚本的
use_rviz:=false，避免同时打开两个 RViz。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("cakebot_description")

    xacro_file = PathJoinSubstitution(
        [package_share, "urdf", "cakebot.urdf.xacro"]
    )
    default_world = PathJoinSubstitution(
        [package_share, "worlds", "cakebot.world"]
    )
    rviz_config_file = PathJoinSubstitution(
        [package_share, "rviz", "cakebot.rviz"]
    )
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )

    world = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world 文件路径。",
    )
    gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="是否启动 Gazebo 图形界面。",
    )
    paused = DeclareLaunchArgument(
        "paused",
        default_value="false",
        description="是否以暂停状态启动仿真。",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="节点是否使用 Gazebo 仿真时间。",
    )
    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否同时启动 RViz。",
    )
    spawn_x = DeclareLaunchArgument(
        "x", default_value="0.0", description="机器人初始 x 坐标。"
    )
    spawn_y = DeclareLaunchArgument(
        "y", default_value="0.0", description="机器人初始 y 坐标。"
    )
    spawn_z = DeclareLaunchArgument(
        "z", default_value="0.01", description="机器人初始 z 坐标。"
    )
    spawn_yaw = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="机器人初始 yaw 角，单位为弧度。"
    )

    robot_description = Command(["xacro ", xacro_file])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "pause": LaunchConfiguration("paused"),
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_cakebot",
        output="screen",
        arguments=[
            "-entity",
            "cakebot",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-Y",
            LaunchConfiguration("yaw"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            world,
            gui,
            paused,
            use_sim_time,
            use_rviz,
            spawn_x,
            spawn_y,
            spawn_z,
            spawn_yaw,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            rviz,
        ]
    )
