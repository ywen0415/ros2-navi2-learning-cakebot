#!/usr/bin/env python3

"""启动三维点云到二维激光扫描的感知适配器。

输入默认为 /lidar/points_raw（sensor_msgs/msg/PointCloud2），输出默认为
/scan（sensor_msgs/msg/LaserScan）。转换的高度带、角度分辨率和量程由
pointcloud_to_laserscan.yaml 控制，用于让现有 SLAM Toolbox 和 AMCL 继续
使用标准 /scan 接口。

本 launch 不启动雷达驱动、Gazebo、SLAM 或 AMCL，也不实现三维避障。
pointcloud_to_laserscan 按需订阅输入点云：只有 /scan 存在订阅者时才会处理数据。
仿真时必须传入 use_sim_time:=true；真机默认使用系统时间。
"""

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
        description="适配器是否使用仿真时钟；Gazebo 中应设为 true。",
    )
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="PointCloud2 到 LaserScan 的投影参数文件。",
    )
    input_cloud_topic = DeclareLaunchArgument(
        "input_cloud_topic",
        default_value="/lidar/points_raw",
        description="输入的 sensor_msgs/msg/PointCloud2 topic。",
    )
    output_scan_topic = DeclareLaunchArgument(
        "output_scan_topic",
        default_value="/scan",
        description="输出的 sensor_msgs/msg/LaserScan topic。",
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
