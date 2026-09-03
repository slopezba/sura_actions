#include "sura_actions/surface_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace sura_actions
{
namespace
{
double sanitizePositive(double value, double fallback)
{
  return value > 0.0 ? value : fallback;
}

std::string modeName(uint8_t mode)
{
  switch (mode) {
    case SurfaceActionServer::SetControlMode::Request::SURFACE:
      return "SURFACE";
    case SurfaceActionServer::SetControlMode::Request::MANUAL:
      return "MANUAL";
    default:
      return std::to_string(mode);
  }
}
}  // namespace

SurfaceActionServer::SurfaceActionServer(const rclcpp::NodeOptions & options)
: Node("surface_action_server", options)
{
  robot_namespace_ = this->declare_parameter<std::string>("robot_namespace", "sura");
  action_name_ = this->declare_parameter<std::string>(
    "action_name", namespacedTopic("actions/surface"));
  set_control_mode_service_ = this->declare_parameter<std::string>(
    "set_control_mode_service", namespacedTopic("control_manager/set_mode"));
  navigator_topic_ = this->declare_parameter<std::string>(
    "navigator_topic", namespacedTopic("navigator/navigation"));
  arbitrator_wrench_topic_ = this->declare_parameter<std::string>(
    "arbitrator_wrench_topic", namespacedTopic("controller/arbitrator/wrench"));
  depth_hold_controller_name_ = this->declare_parameter<std::string>(
    "depth_hold_controller", "depth_hold");
  requester_ = this->declare_parameter<std::string>("requester", "surface");
  priority_ = static_cast<int>(
    std::clamp<int64_t>(this->declare_parameter<int>("priority", 70), 1, 100));
  default_depth_tolerance_ = this->declare_parameter<double>("default_depth_tolerance", 0.10);
  default_timeout_ = this->declare_parameter<double>("default_timeout", 30.0);
  navigator_timeout_ = this->declare_parameter<double>("navigator_timeout", 2.0);
  mode_request_timeout_ = this->declare_parameter<double>("mode_request_timeout", 5.0);
  setpoint_publish_rate_ = this->declare_parameter<double>("setpoint_publish_rate", 10.0);

  set_control_mode_client_ = this->create_client<SetControlMode>(set_control_mode_service_);
  surface_wrench_pub_ = this->create_publisher<SuraWrenchCommandMsg>(
    arbitrator_wrench_topic_, rclcpp::SystemDefaultsQoS());
  navigator_sub_ = this->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](NavigatorMsg::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(navigator_mutex_);
      last_navigator_msg_ = std::move(msg);
      last_navigator_time_ = this->now();
    });

  action_server_ = rclcpp_action::create_server<Surface>(
    this,
    action_name_,
    std::bind(&SurfaceActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SurfaceActionServer::handleCancel, this, std::placeholders::_1),
    std::bind(&SurfaceActionServer::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Surface action server ready on '%s'", action_name_.c_str());
  RCLCPP_INFO(get_logger(), "Using set mode service '%s'", set_control_mode_service_.c_str());
  RCLCPP_INFO(get_logger(), "Using navigator topic '%s'", navigator_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Using arbitrator wrench topic '%s'", arbitrator_wrench_topic_.c_str());
}

rclcpp_action::GoalResponse SurfaceActionServer::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const Surface::Goal> goal)
{
  const double timeout = sanitizePositive(goal->timeout, default_timeout_);
  const double tolerance = sanitizePositive(goal->depth_tolerance, default_depth_tolerance_);

  if (timeout <= 0.0 || tolerance <= 0.0 || goal->surface_force_z >= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Rejecting Surface goal because timeout and depth tolerance must be positive and surface_force_z must be negative.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted Surface goal: target_depth=%.3f tolerance=%.3f timeout=%.3f surface_force_z=%.3f",
    goal->target_depth,
    tolerance,
    timeout,
    goal->surface_force_z);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SurfaceActionServer::handleCancel(
  const std::shared_ptr<GoalHandleSurface>)
{
  RCLCPP_INFO(get_logger(), "Canceling Surface goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SurfaceActionServer::handleAccepted(
  const std::shared_ptr<GoalHandleSurface> goal_handle)
{
  std::thread{std::bind(&SurfaceActionServer::execute, this, std::placeholders::_1), goal_handle}
    .detach();
}

void SurfaceActionServer::execute(const std::shared_ptr<GoalHandleSurface> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const double target_depth = goal->target_depth;
  const double depth_tolerance = sanitizePositive(goal->depth_tolerance, default_depth_tolerance_);
  const double timeout = sanitizePositive(goal->timeout, default_timeout_);
  const double surface_force_z = goal->surface_force_z;
  const auto start_time = this->now();
  const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(timeout);
  const rclcpp::Duration navigator_timeout_duration =
    rclcpp::Duration::from_seconds(sanitizePositive(navigator_timeout_, 2.0));
  const double publish_rate = sanitizePositive(setpoint_publish_rate_, 10.0);

  auto result = std::make_shared<Surface::Result>();
  auto feedback = std::make_shared<Surface::Feedback>();

  std::string mode_error;
  if (!requestControlMode(
      SetControlMode::Request::SURFACE,
      "Surface action goal accepted",
      mode_error))
  {
    double current_depth = std::numeric_limits<double>::quiet_NaN();
    getCurrentDepth(current_depth);
    result->success = false;
    result->message = "Failed to request SURFACE mode: " + mode_error;
    result->final_depth = current_depth;
    goal_handle->abort(result);
    return;
  }

  rclcpp::Rate loop_rate(publish_rate);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      double current_depth = std::numeric_limits<double>::quiet_NaN();
      getCurrentDepth(current_depth);
      requestTerminalMode("Surface action canceled");
      result->success = false;
      result->message = "Surface goal canceled";
      result->final_depth = current_depth;
      goal_handle->canceled(result);
      return;
    }

    double current_depth = std::numeric_limits<double>::quiet_NaN();
    const bool has_depth = getCurrentDepth(current_depth);
    const double depth_error = has_depth ? target_depth - current_depth :
      std::numeric_limits<double>::quiet_NaN();

    feedback->current_depth = current_depth;
    feedback->depth_error = depth_error;
    feedback->time_remaining = std::max(
      0.0,
      timeout - (this->now() - start_time).seconds());
    feedback->state = has_depth ? "surfacing" : "waiting_for_navigation";
    goal_handle->publish_feedback(feedback);

    if (has_depth && std::abs(depth_error) <= depth_tolerance) {
      requestTerminalMode("Surface action succeeded");
      result->success = true;
      result->message = "Surface target reached";
      result->final_depth = current_depth;
      goal_handle->succeed(result);
      return;
    }

    if (has_depth) {
      publishSurfaceWrench(surface_force_z);
    }

    const auto elapsed = this->now() - start_time;
    if (elapsed > timeout_duration) {
      requestTerminalMode("Surface action timed out");
      result->success = false;
      result->message = "Surface goal timed out";
      result->final_depth = current_depth;
      goal_handle->abort(result);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(navigator_mutex_);
      const bool navigator_missing =
        !last_navigator_msg_ && elapsed > navigator_timeout_duration;
      const bool navigator_stale =
        last_navigator_msg_ && (this->now() - last_navigator_time_) > navigator_timeout_duration;
      if (navigator_missing || navigator_stale) {
        requestTerminalMode("Surface action lost navigation");
        result->success = false;
        result->message = navigator_missing ? "Navigator data was not received" :
          "Navigator data timed out";
        result->final_depth = current_depth;
        goal_handle->abort(result);
        return;
      }
    }

    loop_rate.sleep();
  }

  result->success = false;
  result->message = "ROS shutdown while executing Surface goal";
  result->final_depth = std::numeric_limits<double>::quiet_NaN();
  requestTerminalMode("Surface action stopped by ROS shutdown");
  goal_handle->abort(result);
}

bool SurfaceActionServer::requestControlMode(
  uint8_t mode,
  const std::string & reason,
  std::string & error_message)
{
  if (!set_control_mode_client_->wait_for_service(
      std::chrono::duration<double>(sanitizePositive(mode_request_timeout_, 5.0))))
  {
    error_message = "set_mode service is not available";
    return false;
  }

  auto request = std::make_shared<SetControlMode::Request>();
  request->mode = mode;
  request->reason = reason;

  auto future = set_control_mode_client_->async_send_request(request);
  const auto status = future.wait_for(
    std::chrono::duration<double>(sanitizePositive(mode_request_timeout_, 5.0) + 1.0));

  if (status != std::future_status::ready) {
    error_message = "set_mode request timed out for mode " + modeName(mode);
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    error_message = response->message;
    return false;
  }

  return true;
}

void SurfaceActionServer::requestTerminalMode(const std::string & reason)
{
  std::string mode_error;
  if (!requestControlMode(SetControlMode::Request::MANUAL, reason, mode_error)) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to request terminal MANUAL mode: %s",
      mode_error.c_str());
  }
}

void SurfaceActionServer::publishSurfaceWrench(double surface_force_z)
{
  SuraWrenchCommandMsg msg;
  msg.header.stamp = now();
  msg.requester = requester_;
  msg.controller = depth_hold_controller_name_;
  msg.priority = static_cast<uint8_t>(priority_);
  msg.wrench.force.z = surface_force_z;
  surface_wrench_pub_->publish(msg);
}

bool SurfaceActionServer::getCurrentDepth(double & depth) const
{
  std::lock_guard<std::mutex> lock(navigator_mutex_);
  if (!last_navigator_msg_) {
    return false;
  }

  depth = last_navigator_msg_->position.position.z;
  return std::isfinite(depth);
}

std::string SurfaceActionServer::namespacedTopic(const std::string & suffix) const
{
  std::string normalized_namespace = robot_namespace_;
  normalized_namespace.erase(
    std::remove(normalized_namespace.begin(), normalized_namespace.end(), '/'),
    normalized_namespace.end());

  if (normalized_namespace.empty()) {
    return "/" + suffix;
  }

  return "/" + normalized_namespace + "/" + suffix;
}

}  // namespace sura_actions

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sura_actions::SurfaceActionServer>());
  rclcpp::shutdown();
  return 0;
}
