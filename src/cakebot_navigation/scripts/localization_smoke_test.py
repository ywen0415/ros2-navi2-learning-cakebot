#!/usr/bin/env python3

"""Check the cakebot AMCL localization data path."""

# 本脚本用于阶段 8 的可重复验收，不启动 Gazebo 或 AMCL 本身。运行前应由
# gazebo.launch.py 和 localization.launch.py 分别提供传感器与定位系统。
# 脚本依次等待 /map、/scan、/odom，发布 /initialpose，然后确认 AMCL 已发布
# /amcl_pose、项目轨迹节点已发布 /amcl_path，且 map -> odom TF 可用。
#
# 在 Gazebo 中必须通过 --ros-args -p use_sim_time:=true 使用仿真时间。
# initial_x、initial_y、initial_yaw 是地图坐标系中的初始位姿，不应机械地
# 假定为 Gazebo 的世界坐标。require_motion:=true 时，脚本会等待键盘控制
# 使 AMCL 估计位置移动指定距离。成功返回 0，检查失败返回 1，手动中断返回 130。

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class LocalizationSmokeTest(Node):
    """Publish an initial pose and verify the essential AMCL outputs."""

    def __init__(self):
        super().__init__("localization_smoke_test")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.initial_x = self.declare_parameter("initial_x", 0.0).value
        self.initial_y = self.declare_parameter("initial_y", 0.0).value
        self.initial_yaw = self.declare_parameter("initial_yaw", 0.0).value
        self.initial_covariance_xy = self.declare_parameter(
            "initial_covariance_xy", 0.25
        ).value
        self.initial_covariance_yaw = self.declare_parameter(
            "initial_covariance_yaw", 0.0685389
        ).value
        self.timeout_sec = self.declare_parameter("timeout_sec", 20.0).value
        self.require_motion = self.declare_parameter("require_motion", False).value
        self.motion_distance = self.declare_parameter("motion_distance", 0.05).value
        self.motion_timeout_sec = self.declare_parameter(
            "motion_timeout_sec", 15.0
        ).value

        self.map_received = False
        self.scan_received = False
        self.odom_received = False
        self.amcl_pose = None
        self.amcl_path = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, map_qos
        )
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_callback,
            10,
        )
        self.create_subscription(
            Path, "/amcl_path", self._amcl_path_callback, path_qos
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    def _map_callback(self, _message):
        self.map_received = True

    def _scan_callback(self, _message):
        self.scan_received = True

    def _odom_callback(self, _message):
        self.odom_received = True

    def _amcl_pose_callback(self, message):
        self.amcl_pose = message

    def _amcl_path_callback(self, message):
        self.amcl_path = message

    def _spin_until(self, condition, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if condition():
                return True
        return condition()

    def _tf_available(self):
        try:
            return self.tf_buffer.can_transform(
                self.map_frame, self.odom_frame, Time()
            )
        except TransformException:
            return False

    def _initial_pose_message(self):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = self.initial_x
        message.pose.pose.position.y = self.initial_y
        message.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)
        message.pose.covariance[0] = self.initial_covariance_xy
        message.pose.covariance[7] = self.initial_covariance_xy
        message.pose.covariance[35] = self.initial_covariance_yaw
        return message

    def _publish_initial_pose(self):
        message = self._initial_pose_message()
        self.get_logger().info(
            "Publishing initial pose (x=%.3f, y=%.3f, yaw=%.3f) in '%s'"
            % (self.initial_x, self.initial_y, self.initial_yaw, self.map_frame)
        )

        # Repeat briefly so the test is robust to AMCL and DDS discovery timing.
        for _ in range(3):
            message.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_publisher.publish(message)
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)

    def _base_checks_ready(self):
        return (
            self.map_received
            and self.scan_received
            and self.odom_received
            and self.amcl_pose is not None
            and self.amcl_path is not None
            and len(self.amcl_path.poses) > 0
            and self._tf_available()
        )

    def _has_required_frames(self):
        pose_frame = self.amcl_pose.header.frame_id
        path_frame = self.amcl_path.header.frame_id
        return pose_frame == self.map_frame and path_frame == self.map_frame

    def _wait_for_motion(self):
        start_x = self.amcl_pose.pose.pose.position.x
        start_y = self.amcl_pose.pose.pose.position.y

        def moved():
            current_x = self.amcl_pose.pose.pose.position.x
            current_y = self.amcl_pose.pose.pose.position.y
            return math.hypot(current_x - start_x, current_y - start_y) >= self.motion_distance

        return self._spin_until(moved, self.motion_timeout_sec)

    def run(self):
        self.get_logger().info("Waiting for map, scan, and odometry data")

        sensors_ready = self._spin_until(
            lambda: self.map_received and self.scan_received and self.odom_received,
            self.timeout_sec,
        )
        if not sensors_ready:
            self.get_logger().error(
                "Timed out waiting for /map, /scan, and /odom "
                "(map=%s, scan=%s, odom=%s)"
                % (self.map_received, self.scan_received, self.odom_received)
            )
            return 1

        self._publish_initial_pose()
        localization_ready = self._spin_until(self._base_checks_ready, self.timeout_sec)
        if not localization_ready:
            self.get_logger().error(
                "AMCL localization check timed out "
                "(pose=%s, path=%s, path_poses=%s, map_to_odom=%s)"
                % (
                    self.amcl_pose is not None,
                    self.amcl_path is not None,
                    len(self.amcl_path.poses) if self.amcl_path is not None else 0,
                    self._tf_available(),
                )
            )
            return 1

        if not self._has_required_frames():
            self.get_logger().error(
                "Unexpected frames: amcl_pose=%s, amcl_path=%s, expected=%s"
                % (
                    self.amcl_pose.header.frame_id,
                    self.amcl_path.header.frame_id,
                    self.map_frame,
                )
            )
            return 1

        self.get_logger().info(
            "AMCL localization is active: pose frame=%s, path poses=%d, "
            "TF %s -> %s is available"
            % (
                self.amcl_pose.header.frame_id,
                len(self.amcl_path.poses),
                self.map_frame,
                self.odom_frame,
            )
        )

        if self.require_motion:
            self.get_logger().info(
                "Waiting for AMCL pose to move by at least %.3f m; "
                "use the keyboard controller now" % self.motion_distance
            )
            if not self._wait_for_motion():
                self.get_logger().error(
                    "No AMCL motion of %.3f m observed within %.1f seconds"
                    % (self.motion_distance, self.motion_timeout_sec)
                )
                return 1
            self.get_logger().info("AMCL motion and path update detected")

        self.get_logger().info("Localization smoke test PASSED")
        return 0


def main(args=None):
    """Run the localization smoke test and return its result code."""
    rclpy.init(args=args)
    node = LocalizationSmokeTest()
    try:
        return node.run()
    except KeyboardInterrupt:
        node.get_logger().warning("Localization smoke test interrupted")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
