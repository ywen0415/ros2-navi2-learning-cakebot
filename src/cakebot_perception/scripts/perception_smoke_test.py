#!/usr/bin/env python3

"""验证 MID-360 PointCloud2 -> LaserScan 感知适配链路。

本脚本不启动 Gazebo、雷达驱动或点云适配器。运行前必须已有：
  - /lidar/points_raw（PointCloud2）或通过参数指定的等价点云 topic；
  - 投影后的 /scan（LaserScan）；
  - base_link -> laser_link 或参数指定的传感器 TF。

检查内容包括点云 xyz 字段、点数、frame、扫描角度/有效距离数、
点云与扫描频率、TF 可用性、数据新鲜度以及 /scan 是否恰好只有一个发布者。
成功返回 0，验收失败返回 1，用户中断返回 130。本测试只验证建图/
定位接口，不证明三维避障或真机功能安全性。
"""

from collections import deque
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


class PerceptionSmokeTest(Node):
    """Verify the point cloud, projected scan, and sensor TF contract."""

    def __init__(self):
        super().__init__("perception_smoke_test")

        self.input_cloud_topic = self.declare_parameter(
            "input_cloud_topic", "/lidar/points_raw"
        ).value
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.sensor_frame = self.declare_parameter("sensor_frame", "laser_link").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 20.0).value)
        self.min_samples = int(self.declare_parameter("min_samples", 5).value)
        self.min_rate_hz = float(self.declare_parameter("min_rate_hz", 5.0).value)
        self.min_scan_size = int(self.declare_parameter("min_scan_size", 300).value)
        self.min_valid_ranges = int(
            self.declare_parameter("min_valid_ranges", 20).value
        )
        self.require_tf = bool(self.declare_parameter("require_tf", True).value)

        self.cloud_received = False
        self.scan_received = False
        self.cloud_frame = ""
        self.scan_frame = ""
        self.cloud_point_count = 0
        self.cloud_fields = set()
        self.scan_size = 0
        self.scan_valid_count = 0
        self.scan_angle_span = 0.0
        self.last_cloud_wall = None
        self.last_scan_wall = None
        self.cloud_arrivals = deque(maxlen=100)
        self.scan_arrivals = deque(maxlen=100)

        self.create_subscription(
            PointCloud2,
            self.input_cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

    def _cloud_callback(self, message):
        now = time.monotonic()
        self.cloud_received = True
        self.cloud_frame = message.header.frame_id
        self.cloud_point_count = int(message.width) * int(message.height)
        self.cloud_fields = {field.name for field in message.fields}
        self.last_cloud_wall = now
        self.cloud_arrivals.append(now)

    def _scan_callback(self, message):
        now = time.monotonic()
        self.scan_received = True
        self.scan_frame = message.header.frame_id
        self.scan_size = len(message.ranges)
        self.scan_valid_count = sum(
            1
            for value in message.ranges
            if math.isfinite(value)
            and message.range_min <= value <= message.range_max
        )
        self.scan_angle_span = message.angle_max - message.angle_min
        self.last_scan_wall = now
        self.scan_arrivals.append(now)

    @staticmethod
    def _rate(arrivals):
        if len(arrivals) < 2 or arrivals[-1] <= arrivals[0]:
            return 0.0
        return (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])

    def _tf_available(self):
        if self.sensor_frame == self.base_frame:
            return True
        try:
            return self.tf_buffer.can_transform(
                self.base_frame, self.sensor_frame, Time()
            )
        except TransformException:
            return False

    def _wait_for_data(self):
        deadline = time.monotonic() + self.timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            enough_samples = (
                len(self.cloud_arrivals) >= self.min_samples
                and len(self.scan_arrivals) >= self.min_samples
            )
            if self.cloud_received and self.scan_received and enough_samples:
                if not self.require_tf or self._tf_available():
                    return True
        return False

    def _validate(self):
        errors = []

        if not self.cloud_received:
            errors.append(f"no messages received on {self.input_cloud_topic}")
        if not self.scan_received:
            errors.append(f"no messages received on {self.scan_topic}")

        required_fields = {"x", "y", "z"}
        missing_fields = required_fields - self.cloud_fields
        if self.cloud_received and missing_fields:
            errors.append(
                "PointCloud2 is missing fields: " + ", ".join(sorted(missing_fields))
            )
        if self.cloud_received and self.cloud_point_count <= 0:
            errors.append("PointCloud2 contains no points")
        if self.cloud_received and self.cloud_frame != self.sensor_frame:
            errors.append(
                f"PointCloud2 frame is '{self.cloud_frame}', "
                f"expected '{self.sensor_frame}'"
            )

        if self.scan_received and self.scan_frame != self.sensor_frame:
            errors.append(
                f"LaserScan frame is '{self.scan_frame}', "
                f"expected '{self.sensor_frame}'"
            )
        if self.scan_received and self.scan_size < self.min_scan_size:
            errors.append(
                f"LaserScan has {self.scan_size} rays, "
                f"expected at least {self.min_scan_size}"
            )
        if self.scan_received and self.scan_angle_span < 6.0:
            errors.append(
                f"LaserScan angle span is only {self.scan_angle_span:.3f} rad"
            )
        if self.scan_received and self.scan_valid_count < self.min_valid_ranges:
            errors.append(
                f"LaserScan has {self.scan_valid_count} valid ranges, "
                f"expected at least {self.min_valid_ranges}"
            )

        cloud_rate = self._rate(self.cloud_arrivals)
        scan_rate = self._rate(self.scan_arrivals)
        if cloud_rate < self.min_rate_hz:
            errors.append(
                f"PointCloud2 rate is {cloud_rate:.2f} Hz, "
                f"expected at least {self.min_rate_hz:.2f} Hz"
            )
        if scan_rate < self.min_rate_hz:
            errors.append(
                f"LaserScan rate is {scan_rate:.2f} Hz, "
                f"expected at least {self.min_rate_hz:.2f} Hz"
            )

        if self.require_tf and not self._tf_available():
            errors.append(
                f"TF {self.base_frame} -> {self.sensor_frame} is unavailable"
            )

        cloud_publishers = self.count_publishers(self.input_cloud_topic)
        scan_publishers = self.count_publishers(self.scan_topic)
        if cloud_publishers < 1:
            errors.append(f"no publisher found for {self.input_cloud_topic}")
        if scan_publishers != 1:
            errors.append(
                f"expected exactly one publisher for {self.scan_topic}, "
                f"found {scan_publishers}"
            )

        if self.last_cloud_wall is not None:
            if time.monotonic() - self.last_cloud_wall > 1.0:
                errors.append("PointCloud2 stream is stale")
        if self.last_scan_wall is not None:
            if time.monotonic() - self.last_scan_wall > 1.0:
                errors.append("LaserScan stream is stale")

        return errors, cloud_rate, scan_rate, cloud_publishers, scan_publishers

    def run(self):
        self.get_logger().info(
            "Waiting for PointCloud2, LaserScan, and sensor TF data"
        )
        if not self._wait_for_data():
            self.get_logger().error(
                "Timed out after %.1f seconds waiting for perception data "
                "(cloud=%s, scan=%s, cloud_samples=%d, scan_samples=%d)"
                % (
                    self.timeout_sec,
                    self.cloud_received,
                    self.scan_received,
                    len(self.cloud_arrivals),
                    len(self.scan_arrivals),
                )
            )
            return 1

        errors, cloud_rate, scan_rate, cloud_publishers, scan_publishers = (
            self._validate()
        )
        self.get_logger().info(
            "Perception data received: cloud_points=%d, scan_rays=%d, "
            "valid_ranges=%d, cloud_rate=%.2f Hz, scan_rate=%.2f Hz, "
            "publishers=(cloud:%d, scan:%d)"
            % (
                self.cloud_point_count,
                self.scan_size,
                self.scan_valid_count,
                cloud_rate,
                scan_rate,
                cloud_publishers,
                scan_publishers,
            )
        )

        if errors:
            for error in errors:
                self.get_logger().error(error)
            self.get_logger().error("Perception smoke test FAILED")
            return 1

        self.get_logger().info("Perception smoke test PASSED")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSmokeTest()
    try:
        return node.run()
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
