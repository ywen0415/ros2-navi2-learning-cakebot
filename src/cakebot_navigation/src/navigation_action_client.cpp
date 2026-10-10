// C++ client for Nav2's NavigateToPose and NavigateThroughPoses actions.
//
// The poses parameter is a flat list of [x, y, yaw] triples. Select the
// action with action_type=to_pose or action_type=through_poses. A positive
// cancel_after_sec requests cancellation of this client's accepted goal.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{
constexpr char kNavigateToPoseAction[] = "/navigate_to_pose";
constexpr char kNavigateThroughPosesAction[] = "/navigate_through_poses";

enum ExitCode : int
{
  kSuccess = 0,
  kRuntimeError = 1,
  kInvalidParameters = 2,
  kServerUnavailable = 4,
  kGoalSendFailed = 5,
  kGoalRejected = 6,
  kCanceled = 7,
  kAborted = 8,
  kUnknownResult = 9,
};

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}
}  // namespace

class NavigationActionClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using ToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using ThroughPosesGoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  NavigationActionClient()
  : Node("navigation_action_client")
  {
    action_type_ = declare_parameter<std::string>("action_type", "to_pose");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    poses_ = declare_parameter<std::vector<double>>(
      "poses", std::vector<double>{1.0, 0.0, 0.0});
    server_timeout_sec_ = declare_parameter<double>("server_timeout_sec", 5.0);
    feedback_period_sec_ = declare_parameter<double>("feedback_period_sec", 1.0);
    cancel_after_sec_ = declare_parameter<double>("cancel_after_sec", -1.0);

    validate_parameters();

    if (action_type_ == "to_pose") {
      action_name_ = kNavigateToPoseAction;
      to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
    } else {
      action_name_ = kNavigateThroughPosesAction;
      through_poses_client_ =
        rclcpp_action::create_client<NavigateThroughPoses>(this, action_name_);
    }
  }

  bool start()
  {
    RCLCPP_INFO(
      get_logger(), "Waiting up to %.1f seconds for Action server '%s'",
      server_timeout_sec_, action_name_.c_str());

    const auto timeout = std::chrono::duration<double>(server_timeout_sec_);
    const bool server_ready = action_type_ == "to_pose" ?
      to_pose_client_->wait_for_action_server(timeout) :
      through_poses_client_->wait_for_action_server(timeout);

    if (!server_ready) {
      RCLCPP_ERROR(
        get_logger(), "Action server '%s' is unavailable after %.1f seconds",
        action_name_.c_str(), server_timeout_sec_);
      exit_code_ = kServerUnavailable;
      finished_ = true;
      return false;
    }

    try {
      if (action_type_ == "to_pose") {
        send_to_pose_goal();
      } else {
        send_through_poses_goal();
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to send goal to Action server '%s': %s",
        action_name_.c_str(), error.what());
      exit_code_ = kGoalSendFailed;
      finished_ = true;
      return false;
    }
    return true;
  }

  bool finished() const
  {
    return finished_;
  }

  int exit_code() const
  {
    return exit_code_;
  }

private:
  void validate_parameters() const
  {
    if (action_type_ != "to_pose" && action_type_ != "through_poses") {
      throw std::invalid_argument("action_type must be 'to_pose' or 'through_poses'");
    }
    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id must not be empty");
    }
    if (poses_.empty() || poses_.size() % 3 != 0) {
      throw std::invalid_argument("poses must contain one or more [x, y, yaw] triples");
    }
    if (action_type_ == "to_pose" && poses_.size() != 3) {
      throw std::invalid_argument("action_type 'to_pose' requires exactly one [x, y, yaw] triple");
    }
    for (const double value : poses_) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("poses values must all be finite");
      }
    }
    if (!std::isfinite(server_timeout_sec_) || server_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("server_timeout_sec must be finite and greater than zero");
    }
    if (!std::isfinite(feedback_period_sec_) || feedback_period_sec_ < 0.0) {
      throw std::invalid_argument("feedback_period_sec must be finite and non-negative");
    }
    if (!std::isfinite(cancel_after_sec_)) {
      throw std::invalid_argument("cancel_after_sec must be finite");
    }
  }

  geometry_msgs::msg::PoseStamped make_pose(std::size_t offset) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.header.stamp = now();
    pose.pose.position.x = poses_.at(offset);
    pose.pose.position.y = poses_.at(offset + 1);

    const double half_yaw = poses_.at(offset + 2) * 0.5;
    pose.pose.orientation.z = std::sin(half_yaw);
    pose.pose.orientation.w = std::cos(half_yaw);
    return pose;
  }

  void send_to_pose_goal()
  {
    NavigateToPose::Goal goal;
    goal.pose = make_pose(0);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const ToPoseGoalHandle::SharedPtr & goal_handle) {
        if (!goal_handle) {
          handle_goal_rejected();
          return;
        }
        to_pose_goal_handle_ = goal_handle;
        handle_goal_accepted();
      };
    options.feedback_callback =
      [this](
      ToPoseGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        if (!should_log_feedback()) {
          return;
        }
        RCLCPP_INFO(
          get_logger(),
          "Feedback: distance_remaining=%.3f m, navigation_time=%.1f s, "
          "estimated_time_remaining=%.1f s, recoveries=%d",
          static_cast<double>(feedback->distance_remaining),
          duration_seconds(feedback->navigation_time),
          duration_seconds(feedback->estimated_time_remaining),
          static_cast<int>(feedback->number_of_recoveries));
      };
    options.result_callback =
      [this](const ToPoseGoalHandle::WrappedResult & result) {
        handle_result(result.code);
      };

    (void)to_pose_client_->async_send_goal(goal, options);
  }

  void send_through_poses_goal()
  {
    NavigateThroughPoses::Goal goal;
    goal.poses.reserve(poses_.size() / 3);
    for (std::size_t offset = 0; offset < poses_.size(); offset += 3) {
      goal.poses.push_back(make_pose(offset));
    }

    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const ThroughPosesGoalHandle::SharedPtr & goal_handle) {
        if (!goal_handle) {
          handle_goal_rejected();
          return;
        }
        through_poses_goal_handle_ = goal_handle;
        handle_goal_accepted();
      };
    options.feedback_callback =
      [this](
      ThroughPosesGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
        if (!should_log_feedback()) {
          return;
        }
        RCLCPP_INFO(
          get_logger(),
          "Feedback: distance_remaining=%.3f m, poses_remaining=%d, "
          "navigation_time=%.1f s, estimated_time_remaining=%.1f s, recoveries=%d",
          static_cast<double>(feedback->distance_remaining),
          static_cast<int>(feedback->number_of_poses_remaining),
          duration_seconds(feedback->navigation_time),
          duration_seconds(feedback->estimated_time_remaining),
          static_cast<int>(feedback->number_of_recoveries));
      };
    options.result_callback =
      [this](const ThroughPosesGoalHandle::WrappedResult & result) {
        handle_result(result.code);
      };

    (void)through_poses_client_->async_send_goal(goal, options);
  }

  void handle_goal_accepted()
  {
    RCLCPP_INFO(
      get_logger(), "Goal accepted by Action server '%s'", action_name_.c_str());

    if (cancel_after_sec_ > 0.0) {
      const auto delay = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(cancel_after_sec_));
      cancel_timer_ = create_wall_timer(delay, [this]() {request_cancel();});
    }
  }

  void handle_goal_rejected()
  {
    RCLCPP_ERROR(
      get_logger(), "Goal was rejected by Action server '%s'", action_name_.c_str());
    finish(kGoalRejected);
  }

  bool should_log_feedback()
  {
    const auto current_time = std::chrono::steady_clock::now();
    if (!feedback_received_ || feedback_period_sec_ == 0.0 ||
      std::chrono::duration<double>(current_time - last_feedback_time_).count() >=
      feedback_period_sec_)
    {
      feedback_received_ = true;
      last_feedback_time_ = current_time;
      return true;
    }
    return false;
  }

  void request_cancel()
  {
    if (cancel_timer_) {
      cancel_timer_->cancel();
    }
    if (finished_ || cancel_requested_) {
      return;
    }
    cancel_requested_ = true;
    RCLCPP_INFO(get_logger(), "Requesting cancellation of the active goal");

    try {
      if (action_type_ == "to_pose" && to_pose_goal_handle_) {
        to_pose_client_->async_cancel_goal(
          to_pose_goal_handle_,
          [this](const rclcpp_action::Client<NavigateToPose>::CancelResponse::SharedPtr response) {
            handle_cancel_response(response->goals_canceling.empty());
          });
      } else if (action_type_ == "through_poses" && through_poses_goal_handle_) {
        through_poses_client_->async_cancel_goal(
          through_poses_goal_handle_,
          [this](
            const rclcpp_action::Client<NavigateThroughPoses>::CancelResponse::SharedPtr response) {
            handle_cancel_response(response->goals_canceling.empty());
          });
      } else {
        RCLCPP_ERROR(get_logger(), "Cannot cancel because no accepted goal handle is available");
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to request goal cancellation: %s", error.what());
    }
  }

  void handle_cancel_response(bool rejected)
  {
    if (rejected) {
      RCLCPP_WARN(get_logger(), "Action server rejected the cancellation request");
    } else {
      RCLCPP_INFO(get_logger(), "Action server accepted the cancellation request");
    }
  }

  void handle_result(rclcpp_action::ResultCode result_code)
  {
    switch (result_code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Navigation finished with result code: SUCCEEDED");
        finish(kSuccess);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "Navigation finished with result code: CANCELED");
        finish(kCanceled);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Navigation finished with result code: ABORTED");
        finish(kAborted);
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Navigation finished with result code: UNKNOWN");
        finish(kUnknownResult);
        break;
    }
  }

  void finish(int exit_code)
  {
    if (finished_) {
      return;
    }
    exit_code_ = exit_code;
    finished_ = true;
    if (cancel_timer_) {
      cancel_timer_->cancel();
    }
    rclcpp::shutdown();
  }

  std::string action_type_;
  std::string frame_id_;
  std::string action_name_;
  std::vector<double> poses_;
  double server_timeout_sec_{5.0};
  double feedback_period_sec_{1.0};
  double cancel_after_sec_{-1.0};

  rclcpp_action::Client<NavigateToPose>::SharedPtr to_pose_client_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr through_poses_client_;
  ToPoseGoalHandle::SharedPtr to_pose_goal_handle_;
  ThroughPosesGoalHandle::SharedPtr through_poses_goal_handle_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;

  std::chrono::steady_clock::time_point last_feedback_time_{};
  bool feedback_received_{false};
  bool cancel_requested_{false};
  bool finished_{false};
  int exit_code_{kRuntimeError};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<NavigationActionClient> node;
  try {
    node = std::make_shared<NavigationActionClient>();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("navigation_action_client"), "%s", error.what());
    rclcpp::shutdown();
    return kInvalidParameters;
  }

  if (!node->start()) {
    const int exit_code = node->exit_code();
    rclcpp::shutdown();
    return exit_code;
  }

  rclcpp::spin(node);
  const int exit_code = node->finished() ? node->exit_code() : kRuntimeError;
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
