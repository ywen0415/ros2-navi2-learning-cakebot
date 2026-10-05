// AMCL 估计轨迹发布节点。
//
// AMCL 只发布当前定位结果 /amcl_pose，不保留历史路径。本节点订阅
// geometry_msgs/msg/PoseWithCovarianceStamped，按最小平移距离或最小偏航
// 变化筛选后累积为 nav_msgs/msg/Path，并发布到 /amcl_path，供 RViz 的
// Path display 显示。轨迹坐标系继承第一条 AMCL 位姿的 frame_id（通常为
// map），并通过 max_poses 限制内存占用。
//
// 可通过参数 input_topic、output_topic、min_distance、min_angle、max_poses
// 和 clear_on_new_frame 调整接口和采样策略。该轨迹表示 AMCL 的估计历史，
// 不是 Gazebo 真值轨迹；发生定位校正时出现轻微跳变是预期行为。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

double shortest_angular_distance(double from, double to)
{
  constexpr double pi = 3.14159265358979323846;
  double difference = std::fmod(to - from, 2.0 * pi);
  if (difference > pi) {
    difference -= 2.0 * pi;
  } else if (difference < -pi) {
    difference += 2.0 * pi;
  }
  return difference;
}
}  // namespace

class AmclPathPublisher : public rclcpp::Node
{
public:
  AmclPathPublisher()
  : Node("amcl_path_publisher")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/amcl_pose");
    output_topic_ = declare_parameter<std::string>("output_topic", "/amcl_path");
    min_distance_ = declare_parameter<double>("min_distance", 0.02);
    min_angle_ = declare_parameter<double>("min_angle", 0.02);
    max_poses_ = declare_parameter<int>("max_poses", 2000);
    clear_on_new_frame_ = declare_parameter<bool>("clear_on_new_frame", true);

    if (min_distance_ < 0.0) {
      RCLCPP_WARN(get_logger(), "min_distance cannot be negative; using 0.0");
      min_distance_ = 0.0;
    }
    if (min_angle_ < 0.0) {
      RCLCPP_WARN(get_logger(), "min_angle cannot be negative; using 0.0");
      min_angle_ = 0.0;
    }
    if (max_poses_ < 1) {
      RCLCPP_WARN(get_logger(), "max_poses must be positive; using 1");
      max_poses_ = 1;
    }

    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      output_topic_, rclcpp::QoS(1).reliable().transient_local());

    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&AmclPathPublisher::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Accumulating %s into %s (min distance %.3f m, min angle %.3f rad)",
      input_topic_.c_str(), output_topic_.c_str(), min_distance_, min_angle_);
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring AMCL pose without a frame_id");
      return;
    }

    if (path_.header.frame_id.empty()) {
      path_.header.frame_id = message->header.frame_id;
    } else if (path_.header.frame_id != message->header.frame_id) {
      if (!clear_on_new_frame_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring pose in frame '%s'; path frame is '%s'",
          message->header.frame_id.c_str(), path_.header.frame_id.c_str());
        return;
      }
      path_.poses.clear();
      path_.header.frame_id = message->header.frame_id;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = message->header;
    pose.pose = message->pose.pose;

    if (!path_.poses.empty()) {
      const auto & previous_pose = path_.poses.back().pose;
      const double dx = pose.pose.position.x - previous_pose.position.x;
      const double dy = pose.pose.position.y - previous_pose.position.y;
      const double distance = std::hypot(dx, dy);
      const double previous_yaw = yaw_from_quaternion(previous_pose.orientation);
      const double current_yaw = yaw_from_quaternion(pose.pose.orientation);
      const double angle = std::abs(shortest_angular_distance(previous_yaw, current_yaw));

      if (distance < min_distance_ && angle < min_angle_) {
        return;
      }
    }

    path_.poses.push_back(std::move(pose));
    if (path_.poses.size() > static_cast<std::size_t>(max_poses_)) {
      path_.poses.erase(path_.poses.begin());
    }
    path_.header.stamp = message->header.stamp;
    path_publisher_->publish(path_);
  }

  std::string input_topic_;
  std::string output_topic_;
  double min_distance_{0.02};
  double min_angle_{0.02};
  int max_poses_{2000};
  bool clear_on_new_frame_{true};

  nav_msgs::msg::Path path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclPathPublisher>());
  rclcpp::shutdown();
  return 0;
}
