#!/usr/bin/env python3

"""Launch the navigation part of Nav2 for cakebot.

This launch file deliberately starts navigation only.  Gazebo (or the real
robot), the MID-360 point-cloud adapter, and static-map localization are
provided by their own launch files.  In particular, this file does not start
map_server or AMCL, so it can be used together with the existing
``localization.launch.py`` without creating a second ``map -> odom``
transform publisher.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cakebot_share = FindPackageShare("cakebot_navigation")
    nav2_bringup_share = FindPackageShare("nav2_bringup")

    default_params_file = PathJoinSubstitution(
        [cakebot_share, "config", "nav2_params.yaml"]
    )
    default_rviz_config_file = PathJoinSubstitution(
        [cakebot_share, "rviz", "navigation.rviz"]
    )
    nav2_navigation_launch_file = PathJoinSubstitution(
        [nav2_bringup_share, "launch", "navigation_launch.py"]
    )

    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Nav2 navigation parameters for cakebot.",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Whether Nav2 and RViz use the Gazebo clock.",
    )
    autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically configure and activate Nav2 lifecycle nodes.",
    )
    log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level passed to the Nav2 navigation nodes.",
    )
    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start the cakebot navigation RViz.",
    )
    rviz_config_file = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=default_rviz_config_file,
        description="RViz configuration file for navigation.",
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_file),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": LaunchConfiguration("autostart"),
            # Keep the first Nav2 bringup explicit and easy to inspect.  The
            # composed-container variant can be introduced later if needed.
            "use_composition": "False",
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_navigation",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            params_file,
            use_sim_time,
            autostart,
            log_level,
            use_rviz,
            rviz_config_file,
            navigation,
            rviz,
        ]
    )
