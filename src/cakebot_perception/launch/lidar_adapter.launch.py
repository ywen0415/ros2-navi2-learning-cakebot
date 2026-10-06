#!/usr/bin/env python3

"""Project a 3D lidar PointCloud2 stream into the legacy /scan interface."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("cakebot_perception")
    default_params_file = PathJoinSubstitution(
        [package_share, "config", "pointcloud_to_laserscan.yaml"]
    )

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether the adapter uses the simulation clock.",
    )
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="PointCloud2-to-LaserScan parameter file.",
    )
    input_cloud_topic = DeclareLaunchArgument(
        "input_cloud_topic",
        default_value="/lidar/points_raw",
        description="Input sensor_msgs/msg/PointCloud2 topic.",
    )
    output_scan_topic = DeclareLaunchArgument(
        "output_scan_topic",
        default_value="/scan",
        description="Output sensor_msgs/msg/LaserScan topic.",
    )

    adapter = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            },
        ],
        remappings=[
            ("cloud_in", LaunchConfiguration("input_cloud_topic")),
            ("scan", LaunchConfiguration("output_scan_topic")),
        ],
    )

    return LaunchDescription(
        [
            use_sim_time,
            params_file,
            input_cloud_topic,
            output_scan_topic,
            adapter,
        ]
    )
