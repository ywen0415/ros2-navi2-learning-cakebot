// AMCL 估计轨迹发布节点。
//
// AMCL 只发布当前定位结果 /amcl_pose，不保留历史路径。本节点订阅
// geometry_msgs/msg/PoseWithCovarianceStamped，按最小平移距离或最小偏航
// 变化筛选后累积为 nav_msgs/msg/Path，并发布到 /amcl_path，供 RViz 的
// Path display 显示。轨迹坐标系继承第一条 AMCL 位姿的 frame_id（通常为
// map），并通过 max_poses 限制内存占用。
//
// 可通过参数 input_topic、output_topic、min_distance、min_angle、max_poses
// 和 clear_on_new_frame 调整接口和采样策略。save_csv、csv_output_path
// 和 csv_overwrite 用于在节点正常退出时自动保存完整的估计轨迹。该轨迹
// 表示 AMCL 的估计历史，不是 Gazebo 真值轨迹；发生定位校正时出现轻微
// 跳变是预期行为。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <string>
#include <system_error>
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

std::string csv_escape(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (const char character : value) {
    if (character == '"') {
      escaped.push_back('"');
    }
    escaped.push_back(character);
  }
  escaped.push_back('"');
  return escaped;
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
    save_csv_ = declare_parameter<bool>("save_csv", false);
    csv_output_path_ = declare_parameter<std::string>(
      "csv_output_path", "amcl_trajectory.csv");
    csv_overwrite_ = declare_parameter<bool>("csv_overwrite", false);

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

    if (save_csv_) {
      initialize_csv();
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

  bool finalize_csv()
  {
    if (!save_csv_) {
      return true;
    }
    if (csv_finalized_) {
      return csv_finalize_succeeded_;
    }
    csv_finalized_ = true;

    if (!csv_stream_.is_open()) {
      return false;
    }

    csv_stream_.flush();
    csv_stream_.close();
    if (csv_write_failed_ || csv_stream_.fail()) {
      RCLCPP_ERROR(
        get_logger(), "Failed to finish trajectory CSV; partial data remains at '%s'",
        csv_temporary_path_.string().c_str());
      return false;
    }

    if (csv_record_count_ == 0) {
      std::error_code remove_error;
      std::filesystem::remove(csv_temporary_path_, remove_error);
      RCLCPP_WARN(
        get_logger(), "No AMCL poses were recorded; trajectory CSV was not created");
      csv_finalize_succeeded_ = true;
      return true;
    }

    std::error_code rename_error;
    std::filesystem::rename(csv_temporary_path_, csv_final_path_, rename_error);
    if (rename_error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to move trajectory CSV from '%s' to '%s': %s",
        csv_temporary_path_.string().c_str(), csv_final_path_.string().c_str(),
        rename_error.message().c_str());
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "Saved %zu AMCL trajectory poses to '%s'",
      csv_record_count_, csv_final_path_.string().c_str());
    csv_finalize_succeeded_ = true;
    return true;
  }

private:
  void initialize_csv()
  {
    if (csv_output_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "csv_output_path cannot be empty when save_csv is enabled");
      return;
    }

    std::error_code path_error;
    csv_final_path_ = std::filesystem::absolute(csv_output_path_, path_error);
    if (path_error) {
      RCLCPP_ERROR(
        get_logger(), "Cannot resolve trajectory CSV path '%s': %s",
        csv_output_path_.c_str(), path_error.message().c_str());
      return;
    }

    const auto parent_path = csv_final_path_.parent_path();
    if (!parent_path.empty()) {
      std::filesystem::create_directories(parent_path, path_error);
      if (path_error) {
        RCLCPP_ERROR(
          get_logger(), "Cannot create trajectory directory '%s': %s",
          parent_path.string().c_str(), path_error.message().c_str());
        return;
      }
    }

    const bool output_exists = std::filesystem::exists(csv_final_path_, path_error);
    if (path_error) {
      RCLCPP_ERROR(
        get_logger(), "Cannot inspect trajectory CSV path '%s': %s",
        csv_final_path_.string().c_str(), path_error.message().c_str());
      return;
    }
    if (output_exists && !csv_overwrite_) {
      RCLCPP_ERROR(
        get_logger(),
        "Trajectory CSV '%s' already exists; choose another path or set csv_overwrite=true",
        csv_final_path_.string().c_str());
      return;
    }

    csv_temporary_path_ = csv_final_path_;
    csv_temporary_path_ += ".tmp";
    csv_stream_.open(csv_temporary_path_, std::ios::out | std::ios::trunc);
    if (!csv_stream_.is_open()) {
      RCLCPP_ERROR(
        get_logger(), "Cannot open temporary trajectory CSV '%s'",
        csv_temporary_path_.string().c_str());
      return;
    }

    csv_stream_ << "index,stamp_sec,stamp_nanosec,frame_id,x,y,yaw\n";
    csv_stream_ << std::setprecision(17);
    if (!csv_stream_) {
      csv_write_failed_ = true;
      RCLCPP_ERROR(
        get_logger(), "Cannot write trajectory CSV header to '%s'",
        csv_temporary_path_.string().c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(), "AMCL trajectory CSV recording enabled: '%s'",
      csv_final_path_.string().c_str());
  }

  void append_csv_pose(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!save_csv_ || !csv_stream_.is_open() || csv_write_failed_) {
      return;
    }

    csv_stream_ << csv_record_count_ << ',' << pose.header.stamp.sec << ',' <<
      pose.header.stamp.nanosec << ',' << csv_escape(pose.header.frame_id) << ',' <<
      pose.pose.position.x << ',' << pose.pose.position.y << ',' <<
      yaw_from_quaternion(pose.pose.orientation) << '\n';

    if (!csv_stream_) {
      csv_write_failed_ = true;
      RCLCPP_ERROR(
        get_logger(), "Failed while writing trajectory CSV '%s'",
        csv_temporary_path_.string().c_str());
      return;
    }

    ++csv_record_count_;
    if (csv_record_count_ % 100 == 0) {
      csv_stream_.flush();
    }
  }

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

    append_csv_pose(pose);
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
  bool save_csv_{false};
  std::string csv_output_path_;
  bool csv_overwrite_{false};
  bool csv_finalized_{false};
  bool csv_finalize_succeeded_{false};
  bool csv_write_failed_{false};
  std::size_t csv_record_count_{0};
  std::filesystem::path csv_final_path_;
  std::filesystem::path csv_temporary_path_;
  std::ofstream csv_stream_;

  nav_msgs::msg::Path path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AmclPathPublisher>();
  rclcpp::spin(node);
  const bool csv_saved = node->finalize_csv();
  node.reset();
  rclcpp::shutdown();
  return csv_saved ? 0 : 1;
}
