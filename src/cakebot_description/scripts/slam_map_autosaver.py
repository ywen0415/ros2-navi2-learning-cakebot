#!/usr/bin/env python3

"""在 SLAM 退出时将缓存的 /map 保存为 Nav2 兼容地图文件。"""

import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def map_prefix(output_file: str) -> Path:
    """将可选的 .yaml 或 .pgm 后缀转换为地图文件前缀。"""
    prefix = Path(output_file).expanduser()
    if prefix.suffix in {".yaml", ".pgm"}:
        prefix = prefix.with_suffix("")
    return prefix


def yaw_from_quaternion(orientation) -> float:
    """从 geometry_msgs/Quaternion 计算平面 yaw。"""
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def occupancy_to_pgm_value(occupancy: int) -> int:
    """将 ROS 占据概率转换为 Nav2 PGM 灰度值。"""
    if occupancy < 0:
        return 205

    occupancy = max(0, min(100, occupancy))
    return round(254 - occupancy * 254 / 100)


def save_map(map_message: OccupancyGrid, output_file: str) -> tuple[Path, Path]:
    """保存 OccupancyGrid，并返回生成的 YAML 和 PGM 路径。"""
    prefix = map_prefix(output_file)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    yaml_path = prefix.with_suffix(".yaml")
    image_path = prefix.with_suffix(".pgm")

    width = map_message.info.width
    height = map_message.info.height
    if width == 0 or height == 0 or len(map_message.data) != width * height:
        raise ValueError("received /map has invalid dimensions or data length")

    with image_path.open("wb") as image_file:
        image_file.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        for row in range(height - 1, -1, -1):
            start = row * width
            values = map_message.data[start:start + width]
            image_file.write(bytes(occupancy_to_pgm_value(value) for value in values))

    origin = map_message.info.origin
    yaw = yaw_from_quaternion(origin.orientation)
    with yaml_path.open("w", encoding="utf-8") as yaml_file:
        yaml_file.write(
            "image: {}\n"
            "mode: trinary\n"
            "resolution: {:.12g}\n"
            "origin: [{:.12g}, {:.12g}, {:.12g}]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n".format(
                image_path.name,
                map_message.info.resolution,
                origin.position.x,
                origin.position.y,
                yaw,
            )
        )

    return yaml_path, image_path


class SlamMapAutosaver(Node):
    """缓存 transient-local /map，并在节点退出时写入磁盘。"""

    def __init__(self):
        super().__init__("slam_map_autosaver")
        self.declare_parameter("output_file", "maps/slam_map")
        self.output_file = self.get_parameter("output_file").value
        self.latest_map = None

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.get_logger().info(
            f"Autosave enabled. Waiting for /map; output prefix: {self.output_file}"
        )

    def map_callback(self, map_message: OccupancyGrid) -> None:
        self.latest_map = map_message

    def save_latest_map(self) -> None:
        if self.latest_map is None:
            self.get_logger().error(
                "Autosave skipped because no /map message was received."
            )
            return

        try:
            yaml_path, image_path = save_map(self.latest_map, self.output_file)
        except (OSError, ValueError) as error:
            self.get_logger().error(f"Failed to save map: {error}")
            return

        self.get_logger().info(
            f"Saved map automatically: {yaml_path} and {image_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SlamMapAutosaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_latest_map()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
