#!/usr/bin/env python3

"""仅在 RViz 中显示 cakebot 机器人模型。

用途：加载 Xacro、启动 robot_state_publisher 和 joint_state_publisher，
用于检查机器人外形、关节和静态 TF。本脚本不启动 Gazebo、激光雷达、
差速驱动或 SLAM，适合模型显示和 URDF 调试。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("cakebot_description")

    xacro_file = PathJoinSubstitution(
        [package_share, "urdf", "cakebot.urdf.xacro"]
    )
    rviz_config_file = PathJoinSubstitution(
        [package_share, "rviz", "cakebot.rviz"]
    )

    # 这些参数对应 cakebot.urdf.xacro 顶部的可配置位置参数。
    # x 正方向是机器人前方，y 正方向是机器人左方。
    caster_x = DeclareLaunchArgument(
        "caster_x",
        default_value="-0.12",
        description="万向轮相对于 base_link 的 x 位置，负值在后方，正值在前方。",
    )
    caster_y = DeclareLaunchArgument(
        "caster_y",
        default_value="0.0",
        description="万向轮相对于 base_link 的 y 位置。",
    )
    caster_yaw = DeclareLaunchArgument(
        "caster_yaw",
        default_value="0.0",
        description="万向轮绕 z 轴的旋转角度，单位为弧度。",
    )
    wheel_x = DeclareLaunchArgument(
        "wheel_x",
        default_value="0.0",
        description="左右驱动轮共同的 x 位置。",
    )
    wheel_separation = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.22",
        description="左右驱动轮中心之间的距离。",
    )
    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启动 RViz。",
    )

    robot_description = Command(
        [
            "xacro ",
            xacro_file,
            " caster_x:=",
            LaunchConfiguration("caster_x"),
            " caster_y:=",
            LaunchConfiguration("caster_y"),
            " caster_yaw:=",
            LaunchConfiguration("caster_yaw"),
            " wheel_x:=",
            LaunchConfiguration("wheel_x"),
            " wheel_separation:=",
            LaunchConfiguration("wheel_separation"),
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            caster_x,
            caster_y,
            caster_yaw,
            wheel_x,
            wheel_separation,
            use_rviz,
            robot_state_publisher,
            joint_state_publisher,
            rviz,
        ]
    )
