#include "sura_actions/go_to_pose_non_holonomic_action_server.hpp"

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

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

bool isFinitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

bool isFinitePosition(const geometry_msgs::msg::Point & position)
{
  return std::isfinite(position.x) &&
         std::isfinite(position.y) &&
         std::isfinite(position.z);
}

std::string modeName(uint8_t mode)
{
  switch (mode) {
    case GoToPoseNonHolonomicActionServer::SetControlMode::Request::MANUAL:
      return "MANUAL";
    case GoToPoseNonHolonomicActionServer::SetControlMode::Request::BODY_VELOCITY:
      return "BODY_VELOCITY";
    case GoToPoseNonHolonomicActionServer::SetControlMode::Request::HOLD_POSITION:
      return "HOLD_POSITION";
    default:
      return std::to_string(mode);
  }
}
}  // namespace

GoToPoseNonHolonomicActionServer::GoToPoseNonHolonomicActionServer(
  const rclcpp::NodeOptions & options)
: Node("go_to_pose_non_holonomic_action_server", options)
{
  robot_namespace_ = this->declare_parameter<std::string>("robot_namespace", "sura");
  action_name_ = this->declare_parameter<std::string>(
    "action_name", namespacedTopic("actions/go_to_pose_non_holonomic"));
  set_control_mode_service_ = this->declare_parameter<std::string>(
    "set_control_mode_service", namespacedTopic("control_manager/set_mode"));
  navigator_topic_ = this->declare_parameter<std::string>(
    "navigator_topic", namespacedTopic("navigator/navigation"));
  body_velocity_command_topic_ = this->declare_parameter<std::string>(
    "body_velocity_command_topic", namespacedTopic("controller/body_velocity/setpoint"));
  depth_setpoint_topic_ = this->declare_parameter<std::string>(
    "depth_setpoint_topic", namespacedTopic("controller/depth_hold/set_point"));
  target_pose_topic_ = this->declare_parameter<std::string>(
    "target_pose_topic", namespacedTopic("actions/go_to_pose_non_holonomic/target_pose"));
  default_depth_tolerance_ = this->declare_parameter<double>("default_depth_tolerance", 0.10);
  navigator_timeout_ = this->declare_parameter<double>("navigator_timeout", 2.0);
  mode_request_timeout_ = this->declare_parameter<double>("mode_request_timeout", 5.0);
  control_loop_rate_ = this->declare_parameter<double>("control_loop_rate", 15.0);
  gain_x_ = this->declare_parameter<double>("gains.x", 0.4);
  gain_y_ = this->declare_parameter<double>("gains.y", 0.4);
  gain_z_ = this->declare_parameter<double>("gains.z", 0.5);
  gain_yaw_ = this->declare_parameter<double>("gains.yaw", 1.0);
  misalignment_slowdown_yaw_ =
    this->declare_parameter<double>("misalignment_slowdown_yaw", 1.0);

  set_control_mode_client_ = this->create_client<SetControlMode>(set_control_mode_service_);
  target_pose_pub_ = this->create_publisher<PoseStampedMsg>(
    target_pose_topic_, rclcpp::SystemDefaultsQoS());
  body_velocity_pub_ = this->create_publisher<TwistMsg>(
    body_velocity_command_topic_, rclcpp::SystemDefaultsQoS());
  depth_setpoint_pub_ = this->create_publisher<DepthSetPointMsg>(
    depth_setpoint_topic_, rclcpp::SystemDefaultsQoS());
  navigator_sub_ = this->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](NavigatorMsg::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(navigator_mutex_);
      last_navigator_msg_ = std::move(msg);
      last_navigator_time_ = this->now();
    });

  action_server_ = rclcpp_action::create_server<GoToPoseNonHolonomic>(
    this,
    action_name_,
    std::bind(
      &GoToPoseNonHolonomicActionServer::handleGoal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&GoToPoseNonHolonomicActionServer::handleCancel, this, std::placeholders::_1),
    std::bind(&GoToPoseNonHolonomicActionServer::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "GoToPoseNonHolonomic action server ready on '%s'", action_name_.c_str());
  RCLCPP_INFO(get_logger(), "Using set mode service '%s'", set_control_mode_service_.c_str());
  RCLCPP_INFO(get_logger(), "Using navigator topic '%s'", navigator_topic_.c_str());
  RCLCPP_INFO(
    get_logger(), "Using body velocity command topic '%s'",
    body_velocity_command_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Using depth setpoint topic '%s'", depth_setpoint_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Using target pose topic '%s'", target_pose_topic_.c_str());
}

rclcpp_action::GoalResponse GoToPoseNonHolonomicActionServer::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const GoToPoseNonHolonomic::Goal> goal)
{
  if (
    !isFinitePosition(goal->target_pose.pose.position) ||
    !std::isfinite(goal->target_yaw) ||
    goal->max_forward_speed <= 0.0 ||
    goal->max_vertical_speed <= 0.0 ||
    goal->max_yaw_rate <= 0.0 ||
    goal->position_tolerance <= 0.0 ||
    goal->yaw_tolerance <= 0.0 ||
    goal->slowdown_distance < 0.0 ||
    goal->timeout <= 0.0)
  {
    RCLCPP_WARN(
      get_logger(),
      "Rejecting GoToPoseNonHolonomic goal because target position and yaw must be finite, speed/rate/tolerances/timeout must be positive, and slowdown_distance must be non-negative.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted GoToPoseNonHolonomic goal: target=(%.3f, %.3f, %.3f) target_yaw=%.3f allow_holonomic=%s max_forward=%.3f max_vertical=%.3f max_yaw_rate=%.3f timeout=%.3f",
    goal->target_pose.pose.position.x,
    goal->target_pose.pose.position.y,
    goal->target_pose.pose.position.z,
    goal->target_yaw,
    goal->allow_holonomic ? "true" : "false",
    goal->max_forward_speed,
    goal->max_vertical_speed,
    goal->max_yaw_rate,
    goal->timeout);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GoToPoseNonHolonomicActionServer::handleCancel(
  const std::shared_ptr<GoalHandleGoToPoseNonHolonomic>)
{
  RCLCPP_INFO(get_logger(), "Canceling GoToPoseNonHolonomic goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GoToPoseNonHolonomicActionServer::handleAccepted(
  const std::shared_ptr<GoalHandleGoToPoseNonHolonomic> goal_handle)
{
  std::thread{
    std::bind(&GoToPoseNonHolonomicActionServer::execute, this, std::placeholders::_1),
    goal_handle}.detach();
}

void GoToPoseNonHolonomicActionServer::execute(
  const std::shared_ptr<GoalHandleGoToPoseNonHolonomic> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const auto start_time = this->now();
  const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(goal->timeout);
  const rclcpp::Duration navigator_timeout_duration =
    rclcpp::Duration::from_seconds(sanitizePositive(navigator_timeout_, 2.0));
  const double loop_rate_hz = sanitizePositive(control_loop_rate_, 15.0);
  const double depth_tolerance = sanitizePositive(default_depth_tolerance_, goal->position_tolerance);
  const double misalignment_slowdown_yaw =
    sanitizePositive(misalignment_slowdown_yaw_, goal->yaw_tolerance);
  const double slowdown_distance =
    sanitizePositive(goal->slowdown_distance, goal->position_tolerance);
  bool final_approach_active = false;

  auto result = std::make_shared<GoToPoseNonHolonomic::Result>();
  auto feedback = std::make_shared<GoToPoseNonHolonomic::Feedback>();

  std::string mode_error;
  if (!requestControlMode(
      SetControlMode::Request::BODY_VELOCITY,
      "GoToPoseNonHolonomic action goal accepted",
      mode_error))
  {
    result->success = false;
    result->message = "Failed to request BODY_VELOCITY mode: " + mode_error;
    goal_handle->abort(result);
    return;
  }

  publishTargetPose(*goal);

  rclcpp::Rate loop_rate(loop_rate_hz);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      publishZeroVelocity();
      result->success = false;
      result->message = "GoToPoseNonHolonomic goal canceled";
      goal_handle->canceled(result);
      return;
    }

    NavigatorMsg navigator_msg;
    rclcpp::Time navigator_stamp;
    const bool has_navigation = getNavigatorSnapshot(navigator_msg, navigator_stamp);
    const auto now = this->now();
    const auto elapsed = now - start_time;

    if (!has_navigation) {
      if (elapsed > navigator_timeout_duration) {
        publishZeroVelocity();
        result->success = false;
        result->message = "Navigator data was not received";
        goal_handle->abort(result);
        return;
      }

      feedback->state = "waiting_for_navigation";
      feedback->current_x = std::numeric_limits<double>::quiet_NaN();
      feedback->current_y = std::numeric_limits<double>::quiet_NaN();
      feedback->current_z = std::numeric_limits<double>::quiet_NaN();
      feedback->current_yaw = std::numeric_limits<double>::quiet_NaN();
      feedback->distance_to_goal = std::numeric_limits<double>::quiet_NaN();
      feedback->yaw_error = std::numeric_limits<double>::quiet_NaN();
      feedback->commanded_forward_speed = 0.0;
      feedback->commanded_lateral_speed = 0.0;
      feedback->commanded_vertical_speed = 0.0;
      feedback->commanded_yaw_rate = 0.0;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
      continue;
    }

    if ((now - navigator_stamp) > navigator_timeout_duration) {
      publishZeroVelocity();
      result->success = false;
      result->message = "Navigator data timed out";
      goal_handle->abort(result);
      return;
    }

    const auto & current_position = navigator_msg.position.position;
    const auto & target_position = goal->target_pose.pose.position;
    const double error_x = target_position.x - current_position.x;
    const double error_y = target_position.y - current_position.y;
    const double distance_to_goal = std::hypot(error_x, error_y);
    const double current_yaw = quaternionToYaw(navigator_msg.position.orientation);
    final_approach_active = final_approach_active || distance_to_goal <= slowdown_distance;
    const double approach_yaw = final_approach_active ? goal->target_yaw : std::atan2(error_y, error_x);
    const double yaw_error = normalizeAngle(approach_yaw - current_yaw);
    const double yaw_rate =
      std::clamp(gain_yaw_ * yaw_error, -goal->max_yaw_rate, goal->max_yaw_rate);
    const double depth_error = target_position.z - current_position.z;
    const double vertical_speed =
      std::clamp(gain_z_ * depth_error, -goal->max_vertical_speed, goal->max_vertical_speed);

    double forward_speed = goal->max_forward_speed;
    double lateral_speed = 0.0;
    if (distance_to_goal < slowdown_distance) {
      forward_speed = goal->max_forward_speed * distance_to_goal / slowdown_distance;
    }

    if (goal->allow_holonomic && final_approach_active) {
      const double cos_yaw = std::cos(current_yaw);
      const double sin_yaw = std::sin(current_yaw);
      const double body_error_x = cos_yaw * error_x + sin_yaw * error_y;
      const double body_error_y = -sin_yaw * error_x + cos_yaw * error_y;
      forward_speed = std::clamp(
        gain_x_ * body_error_x,
        -goal->max_forward_speed,
        goal->max_forward_speed);
      lateral_speed = std::clamp(
        gain_y_ * body_error_y,
        -goal->max_forward_speed,
        goal->max_forward_speed);
    } else {
      const double yaw_alignment_scale =
        std::clamp(1.0 - std::abs(yaw_error) / misalignment_slowdown_yaw, 0.0, 1.0);
      forward_speed *= yaw_alignment_scale;
    }

    publishBodyVelocity(forward_speed, lateral_speed, vertical_speed, yaw_rate);
    publishDepthSetpoint(target_position.z);
    publishTargetPose(*goal);

    feedback->current_x = current_position.x;
    feedback->current_y = current_position.y;
    feedback->current_z = current_position.z;
    feedback->current_yaw = current_yaw;
    feedback->distance_to_goal = distance_to_goal;
    feedback->yaw_error = yaw_error;
    feedback->commanded_forward_speed = forward_speed;
    feedback->commanded_lateral_speed = lateral_speed;
    feedback->commanded_vertical_speed = vertical_speed;
    feedback->commanded_yaw_rate = yaw_rate;
    feedback->state = final_approach_active ? "final_approach" : "moving";
    goal_handle->publish_feedback(feedback);

    if (
      distance_to_goal < goal->position_tolerance &&
      std::abs(depth_error) < depth_tolerance &&
      std::abs(yaw_error) < goal->yaw_tolerance)
    {
      publishZeroVelocity();
      result->success = true;
      result->message = "GoToPoseNonHolonomic target reached";
      goal_handle->succeed(result);
      return;
    }

    if (elapsed > timeout_duration) {
      publishZeroVelocity();
      result->success = false;
      result->message = "GoToPoseNonHolonomic goal timed out";
      goal_handle->abort(result);
      return;
    }

    loop_rate.sleep();
  }

  publishZeroVelocity();
  result->success = false;
  result->message = "ROS shutdown while executing GoToPoseNonHolonomic goal";
  goal_handle->abort(result);
}

bool GoToPoseNonHolonomicActionServer::requestControlMode(
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

void GoToPoseNonHolonomicActionServer::requestHoldPosition(const std::string & reason)
{
  std::string mode_error;
  if (!requestControlMode(
      SetControlMode::Request::MANUAL,
      reason + " - reset controllers before HOLD_POSITION",
      mode_error))
  {
    RCLCPP_WARN(
      get_logger(),
      "Failed to request terminal MANUAL reset before HOLD_POSITION: %s",
      mode_error.c_str());
  }

  mode_error.clear();
  if (!requestControlMode(SetControlMode::Request::HOLD_POSITION, reason, mode_error)) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to request terminal HOLD_POSITION mode: %s",
      mode_error.c_str());
  }
}

void GoToPoseNonHolonomicActionServer::publishZeroVelocity()
{
  publishBodyVelocity(0.0, 0.0, 0.0, 0.0);
}

void GoToPoseNonHolonomicActionServer::publishBodyVelocity(
  double forward_speed,
  double lateral_speed,
  double vertical_speed,
  double yaw_rate)
{
  TwistMsg msg;
  msg.linear.x = forward_speed;
  msg.linear.y = lateral_speed;
  msg.linear.z = vertical_speed;
  msg.angular.z = yaw_rate;
  body_velocity_pub_->publish(msg);
}

void GoToPoseNonHolonomicActionServer::publishDepthSetpoint(double target_depth)
{
  DepthSetPointMsg msg;
  msg.position.x = std::numeric_limits<double>::quiet_NaN();
  msg.position.y = std::numeric_limits<double>::quiet_NaN();
  msg.position.z = target_depth;
  msg.rpy.x = std::numeric_limits<double>::quiet_NaN();
  msg.rpy.y = std::numeric_limits<double>::quiet_NaN();
  msg.rpy.z = std::numeric_limits<double>::quiet_NaN();
  depth_setpoint_pub_->publish(msg);
}

void GoToPoseNonHolonomicActionServer::publishTargetPose(
  const GoToPoseNonHolonomic::Goal & goal)
{
  PoseStampedMsg msg = goal.target_pose;
  msg.header.stamp = this->now();
  msg.pose.orientation = yawToQuaternion(goal.target_yaw);
  target_pose_pub_->publish(msg);
}

bool GoToPoseNonHolonomicActionServer::getNavigatorSnapshot(
  NavigatorMsg & navigator_msg,
  rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(navigator_mutex_);
  if (!last_navigator_msg_) {
    return false;
  }

  navigator_msg = *last_navigator_msg_;
  stamp = last_navigator_time_;
  return isFinitePose(navigator_msg.position);
}

std::string GoToPoseNonHolonomicActionServer::namespacedTopic(const std::string & suffix) const
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
  rclcpp::spin(std::make_shared<sura_actions::GoToPoseNonHolonomicActionServer>());
  rclcpp::shutdown();
  return 0;
}
