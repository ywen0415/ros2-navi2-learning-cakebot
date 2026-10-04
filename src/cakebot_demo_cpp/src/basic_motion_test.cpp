/*
 * basic_motion_test.cpp
 *
 * 用途：
 *   这是 cakebot 的基础运动测试节点，用于验证 Gazebo 仿真中的
 *   /cmd_vel、/odom 和 /scan 接口是否正常工作。
 *
 * 行为：
 *   1. 等待接收到 /odom 和 /scan；
 *   2. 等待一段启动延时；
 *   3. 低速直线前进；
 *   4. 停止一段时间；
 *   5. 原地旋转；
 *   6. 停止并输出测试结果。
 *
 * 安全机制：
 *   - 前方安全距离内检测到障碍物时立即停止；
 *   - /scan 数据超时后立即停止；
 *   - 测试结束后持续发布零速度，保持机器人停止。
 *
 * 注意事项：
 *   - 本节点会自动控制机器人运动，运行前请确认机器人前方有足够空间；
 *   - 默认速度和运动时间仅适用于当前 Gazebo 仿真，真机使用前必须重新评估；
 *   - 本节点用于基础接口联调，不包含 Nav2、路径规划或完整避障功能。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class BasicMotionTest : public rclcpp::Node
{
public:
  BasicMotionTest()
  : Node("basic_motion_test"), state_(State::WAIT_FOR_SENSORS)
  {
    forward_speed_ = declare_parameter("forward_speed", 0.10);
    angular_speed_ = declare_parameter("angular_speed", 0.30);
    forward_duration_ = declare_parameter("forward_duration", 3.0);
    rotate_duration_ = declare_parameter("rotate_duration", 2.0);
    stop_duration_ = declare_parameter("stop_duration", 1.0);
    start_delay_ = declare_parameter("start_delay", 2.0);
    safety_distance_ = declare_parameter("safety_distance", 0.45);
    sensor_timeout_ = declare_parameter("sensor_timeout", 0.5);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr) {
        if (!odom_received_) {
          RCLCPP_INFO(get_logger(), "Received /odom.");
        }
        odom_received_ = true;
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!scan_received_) {
          RCLCPP_INFO(get_logger(), "Received /scan.");
        }
        scan_received_ = true;
        last_scan_ = msg;
        last_scan_time_ = std::chrono::steady_clock::now();
      });

    timer_ = create_wall_timer(100ms, [this]() { on_timer(); });

    RCLCPP_INFO(
      get_logger(),
      "Basic motion test started. Waiting for /odom and /scan before moving.");
  }

private:
  enum class State
  {
    WAIT_FOR_SENSORS,
    START_DELAY,
    FORWARD,
    STOP_AFTER_FORWARD,
    ROTATE,
    FINISHED
  };

  void on_timer()
  {
    const auto now = std::chrono::steady_clock::now();

    if (state_ == State::WAIT_FOR_SENSORS) {
      publish_stop();
      if (odom_received_ && scan_received_) {
        change_state(State::START_DELAY);
        RCLCPP_INFO(
          get_logger(), "Sensors are ready. Starting in %.1f seconds.", start_delay_);
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for sensors: /odom=%s, /scan=%s.",
          odom_received_ ? "ready" : "missing",
          scan_received_ ? "ready" : "missing");
      }
      return;
    }

    if (state_ == State::START_DELAY) {
      publish_stop();
      if (elapsed_seconds(now) >= start_delay_) {
        change_state(State::FORWARD);
        RCLCPP_INFO(get_logger(), "Test step 1/2: moving forward.");
      }
      return;
    }

    if (state_ == State::FORWARD) {
      if (sensor_data_is_stale(now)) {
        finish_with_warning("/scan timed out while moving");
        return;
      }
      if (front_obstacle_detected()) {
        finish_with_warning("obstacle detected within the safety distance");
        return;
      }
      if (elapsed_seconds(now) >= forward_duration_) {
        change_state(State::STOP_AFTER_FORWARD);
        RCLCPP_INFO(get_logger(), "Forward motion complete. Stopping briefly.");
        publish_stop();
      } else {
        publish_velocity(forward_speed_, 0.0);
      }
      return;
    }

    if (state_ == State::STOP_AFTER_FORWARD) {
      publish_stop();
      if (elapsed_seconds(now) >= stop_duration_) {
        change_state(State::ROTATE);
        RCLCPP_INFO(get_logger(), "Test step 2/2: rotating in place.");
      }
      return;
    }

    if (state_ == State::ROTATE) {
      if (sensor_data_is_stale(now)) {
        finish_with_warning("/scan timed out while rotating");
        return;
      }
      if (elapsed_seconds(now) >= rotate_duration_) {
        state_ = State::FINISHED;
        publish_stop();
        RCLCPP_INFO(get_logger(), "Basic motion test PASSED. Robot is stopped.");
      } else {
        publish_velocity(0.0, angular_speed_);
      }
      return;
    }

    publish_stop();
  }

  void change_state(State new_state)
  {
    state_ = new_state;
    state_start_time_ = std::chrono::steady_clock::now();
  }

  double elapsed_seconds(const std::chrono::steady_clock::time_point & now) const
  {
    return std::chrono::duration<double>(now - state_start_time_).count();
  }

  bool sensor_data_is_stale(const std::chrono::steady_clock::time_point & now) const
  {
    return std::chrono::duration<double>(now - last_scan_time_).count() > sensor_timeout_;
  }

  bool front_obstacle_detected() const
  {
    if (!last_scan_) {
      return true;
    }

    constexpr double front_half_angle = 0.52;  // approximately +/- 30 degrees
    for (std::size_t i = 0; i < last_scan_->ranges.size(); ++i) {
      const double angle = last_scan_->angle_min +
        static_cast<double>(i) * last_scan_->angle_increment;
      const float range = last_scan_->ranges[i];

      if (std::abs(angle) <= front_half_angle &&
        std::isfinite(range) && range >= last_scan_->range_min &&
        range <= last_scan_->range_max && range < safety_distance_)
      {
        return true;
      }
    }
    return false;
  }

  void finish_with_warning(const std::string & reason)
  {
    state_ = State::FINISHED;
    publish_stop();
    RCLCPP_WARN(get_logger(), "Test stopped safely: %s.", reason.c_str());
  }

  void publish_velocity(double linear_x, double angular_z)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear_x;
    command.angular.z = angular_z;
    cmd_vel_pub_->publish(command);
  }

  void publish_stop()
  {
    publish_velocity(0.0, 0.0);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  State state_;
  std::chrono::steady_clock::time_point state_start_time_ =
    std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_scan_time_ =
    std::chrono::steady_clock::now();

  bool odom_received_{false};
  bool scan_received_{false};

  double forward_speed_;
  double angular_speed_;
  double forward_duration_;
  double rotate_duration_;
  double stop_duration_;
  double start_delay_;
  double safety_distance_;
  double sensor_timeout_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicMotionTest>());
  rclcpp::shutdown();
  return 0;
}
