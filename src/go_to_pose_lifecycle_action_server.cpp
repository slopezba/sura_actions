#include "sura_actions/go_to_pose_lifecycle_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace sura_actions
{
namespace
{
template<typename T>
T getOrDeclareParameter(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & name,
  const T & default_value)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }

  return node.declare_parameter<T>(name, default_value);
}

double sanitizePositive(double value, double fallback)
{
  return value > 0.0 ? value : fallback;
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double quaternionNorm(const geometry_msgs::msg::Quaternion & q)
{
  return std::sqrt((q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w));
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

bool isValidQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::isfinite(q.x) &&
         std::isfinite(q.y) &&
         std::isfinite(q.z) &&
         std::isfinite(q.w) &&
         quaternionNorm(q) > 1.0e-6;
}

std::string modeName(uint8_t mode)
{
  switch (mode) {
    case GoToPoseLifecycleActionServer::SetControlMode::Request::MANUAL:
      return "MANUAL";
    case GoToPoseLifecycleActionServer::SetControlMode::Request::BODY_VELOCITY:
      return "BODY_VELOCITY";
    case GoToPoseLifecycleActionServer::SetControlMode::Request::HOLD_POSITION:
      return "HOLD_POSITION";
    default:
      return std::to_string(mode);
  }
}
}  // namespace

GoToPoseLifecycleActionServer::GoToPoseLifecycleActionServer(
  const rclcpp::NodeOptions & options)
: LifecycleNode("go_to_pose_lifecycle_action_node", options)
{
}

GoToPoseLifecycleActionServer::~GoToPoseLifecycleActionServer()
{
  cleanupResources();
}

GoToPoseLifecycleActionServer::CallbackReturn GoToPoseLifecycleActionServer::on_configure(
  const rclcpp_lifecycle::State &)
{
  robot_namespace_ = getOrDeclareParameter<std::string>(*this, "robot_namespace", "sura");
  frame_id_ = getOrDeclareParameter<std::string>(*this, "frame_id", "world_ned");
  action_name_ = getOrDeclareParameter<std::string>(
    *this, "action_name", namespacedTopic("actions/go_to_pose"));
  set_control_mode_service_ = getOrDeclareParameter<std::string>(
    *this, "set_control_mode_service", namespacedTopic("control_manager/set_mode"));
  navigator_topic_ = getOrDeclareParameter<std::string>(
    *this, "navigator_topic", namespacedTopic("navigator/navigation"));
  body_velocity_command_topic_ = getOrDeclareParameter<std::string>(
    *this, "body_velocity_command_topic", namespacedTopic("controller/body_velocity/setpoint"));
  depth_setpoint_topic_ = getOrDeclareParameter<std::string>(
    *this, "depth_setpoint_topic", namespacedTopic("controller/depth_hold/set_point"));
  target_pose_topic_ = getOrDeclareParameter<std::string>(
    *this, "target_pose_topic", namespacedTopic("actions/go_to_pose/target_pose"));
  interactive_marker_namespace_ = getOrDeclareParameter<std::string>(
    *this, "interactive_marker_namespace", namespacedTopic("actions/go_to_pose/interactive_marker"));
  default_holonomic_ = getOrDeclareParameter<bool>(*this, "defaults.holonomic", false);
  default_max_forward_speed_ =
    getOrDeclareParameter<double>(*this, "defaults.max_forward_speed", 0.3);
  default_max_vertical_speed_ =
    getOrDeclareParameter<double>(*this, "defaults.max_vertical_speed", 0.3);
  default_max_yaw_rate_ = getOrDeclareParameter<double>(*this, "defaults.max_yaw_rate", 0.5);
  default_position_tolerance_ =
    getOrDeclareParameter<double>(*this, "defaults.position_tolerance", 0.2);
  default_yaw_tolerance_ = getOrDeclareParameter<double>(*this, "defaults.yaw_tolerance", 0.2);
  default_slowdown_distance_ =
    getOrDeclareParameter<double>(*this, "defaults.slowdown_distance", 0.5);
  default_timeout_ = getOrDeclareParameter<double>(*this, "defaults.timeout", 60.0);
  default_depth_tolerance_ =
    getOrDeclareParameter<double>(*this, "default_depth_tolerance", 0.10);
  navigator_timeout_ = getOrDeclareParameter<double>(*this, "navigator_timeout", 2.0);
  mode_request_timeout_ = getOrDeclareParameter<double>(*this, "mode_request_timeout", 5.0);
  control_loop_rate_ = getOrDeclareParameter<double>(*this, "control_loop_rate", 15.0);
  gain_x_ = getOrDeclareParameter<double>(*this, "gains.x", 0.4);
  gain_y_ = getOrDeclareParameter<double>(*this, "gains.y", 0.4);
  gain_z_ = getOrDeclareParameter<double>(*this, "gains.z", 1.0);
  gain_yaw_ = getOrDeclareParameter<double>(*this, "gains.yaw", 1.0);
  misalignment_slowdown_yaw_ =
    getOrDeclareParameter<double>(*this, "misalignment_slowdown_yaw", 1.0);

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

  interactive_marker_server_ = std::make_unique<InteractiveMarkerServer>(
    interactive_marker_namespace_,
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_topics_interface(),
    this->get_node_services_interface());

  planned_pose_.header.frame_id = frame_id_;
  planned_pose_.pose.orientation.w = 1.0;
  has_planned_pose_ = false;
  rebuildInteractiveMarker();

  action_server_ = rclcpp_action::create_server<GoToPose>(
    this,
    action_name_,
    std::bind(
      &GoToPoseLifecycleActionServer::handleGoal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&GoToPoseLifecycleActionServer::handleCancel, this, std::placeholders::_1),
    std::bind(&GoToPoseLifecycleActionServer::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "GoToPose lifecycle action server configured");
  RCLCPP_INFO(get_logger(), "Planning marker namespace '%s'", interactive_marker_namespace_.c_str());
  RCLCPP_INFO(get_logger(), "Action server '%s'", action_name_.c_str());
  return CallbackReturn::SUCCESS;
}

GoToPoseLifecycleActionServer::CallbackReturn GoToPoseLifecycleActionServer::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "GoToPose active: action goals are accepted");
  return CallbackReturn::SUCCESS;
}

GoToPoseLifecycleActionServer::CallbackReturn GoToPoseLifecycleActionServer::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  requestStop();
  joinExecutionThread();
  publishZeroVelocity();
  RCLCPP_INFO(get_logger(), "GoToPose inactive: marker planning is enabled");
  return CallbackReturn::SUCCESS;
}

GoToPoseLifecycleActionServer::CallbackReturn GoToPoseLifecycleActionServer::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

GoToPoseLifecycleActionServer::CallbackReturn GoToPoseLifecycleActionServer::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse GoToPoseLifecycleActionServer::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const GoToPose::Goal> goal)
{
  if (!isActive()) {
    RCLCPP_WARN(get_logger(), "Rejecting GoToPose goal because the node is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (isExecuting()) {
    RCLCPP_WARN(get_logger(), "Rejecting GoToPose goal because another goal is executing");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto resolved_goal = resolveGoalDefaults(*goal);
  if (
    resolved_goal.max_forward_speed <= 0.0 ||
    resolved_goal.max_vertical_speed <= 0.0 ||
    resolved_goal.max_yaw_rate <= 0.0 ||
    resolved_goal.position_tolerance <= 0.0 ||
    resolved_goal.yaw_tolerance <= 0.0 ||
    resolved_goal.slowdown_distance < 0.0 ||
    resolved_goal.timeout <= 0.0)
  {
    RCLCPP_WARN(
      get_logger(),
      "Rejecting GoToPose goal because speed/rate/tolerances/timeout must be positive and slowdown_distance must be non-negative.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  PoseStampedMsg target_pose;
  double target_yaw = 0.0;
  std::string error_message;
  if (!getGoalPose(resolved_goal, target_pose, target_yaw, error_message)) {
    RCLCPP_WARN(get_logger(), "Rejecting GoToPose goal: %s", error_message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted GoToPose goal: target=(%.3f, %.3f, %.3f) target_yaw=%.3f holonomic=%s timeout=%.3f",
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z,
    target_yaw,
    resolved_goal.holonomic ? "true" : "false",
    resolved_goal.timeout);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GoToPoseLifecycleActionServer::handleCancel(
  const std::shared_ptr<GoalHandleGoToPose>)
{
  requestStop();
  RCLCPP_INFO(get_logger(), "Canceling GoToPose goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GoToPoseLifecycleActionServer::handleAccepted(
  const std::shared_ptr<GoalHandleGoToPose> goal_handle)
{
  if (!tryStartExecution()) {
    auto result = std::make_shared<GoToPose::Result>();
    result->success = false;
    result->message = "Another GoToPose goal is already executing";
    goal_handle->abort(result);
    return;
  }

  joinExecutionThread();
  execution_thread_ = std::thread{
    std::bind(&GoToPoseLifecycleActionServer::execute, this, std::placeholders::_1),
    goal_handle};
}

void GoToPoseLifecycleActionServer::execute(
  const std::shared_ptr<GoalHandleGoToPose> goal_handle)
{
  const auto finish = [this]() {
      finishExecution();
      scheduleDeactivateAfterGoal();
    };

  const auto goal = resolveGoalDefaults(*goal_handle->get_goal());
  PoseStampedMsg target_pose;
  double target_yaw = 0.0;
  std::string goal_error;
  if (!getGoalPose(goal, target_pose, target_yaw, goal_error)) {
    auto result = std::make_shared<GoToPose::Result>();
    result->success = false;
    result->message = goal_error;
    goal_handle->abort(result);
    finish();
    return;
  }

  const auto start_time = this->now();
  const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(goal.timeout);
  const rclcpp::Duration navigator_timeout_duration =
    rclcpp::Duration::from_seconds(sanitizePositive(navigator_timeout_, 2.0));
  const double loop_rate_hz = sanitizePositive(control_loop_rate_, 15.0);
  const double depth_tolerance = sanitizePositive(default_depth_tolerance_, goal.position_tolerance);
  const double misalignment_slowdown_yaw =
    sanitizePositive(misalignment_slowdown_yaw_, goal.yaw_tolerance);
  const double slowdown_distance =
    sanitizePositive(goal.slowdown_distance, goal.position_tolerance);
  bool final_approach_active = false;

  auto result = std::make_shared<GoToPose::Result>();
  auto feedback = std::make_shared<GoToPose::Feedback>();

  std::string mode_error;
  if (!requestControlMode(
      SetControlMode::Request::BODY_VELOCITY,
      "GoToPose action goal accepted",
      mode_error))
  {
    result->success = false;
    result->message = "Failed to request BODY_VELOCITY mode: " + mode_error;
    goal_handle->abort(result);
    finish();
    return;
  }

  {
    std::lock_guard<std::mutex> lock(planned_pose_mutex_);
    planned_pose_ = target_pose;
    planned_pose_.pose.orientation = yawToQuaternion(target_yaw);
    has_planned_pose_ = true;
  }
  publishPlannedPose();

  rclcpp::Rate loop_rate(loop_rate_hz);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling() || stop_requested_) {
      publishZeroVelocity();
      result->success = false;
      result->message = "GoToPose goal canceled";
      goal_handle->canceled(result);
      finish();
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
        finish();
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
      finish();
      return;
    }

    const auto & current_position = navigator_msg.position.position;
    const auto & target_position = target_pose.pose.position;
    const double error_x = target_position.x - current_position.x;
    const double error_y = target_position.y - current_position.y;
    const double distance_to_goal = std::hypot(error_x, error_y);
    const double current_yaw = quaternionToYaw(navigator_msg.position.orientation);
    final_approach_active = final_approach_active || distance_to_goal <= slowdown_distance;
    const bool use_holonomic_control = goal.holonomic || final_approach_active;
    const double approach_yaw = use_holonomic_control ? target_yaw : std::atan2(error_y, error_x);
    const double yaw_error = normalizeAngle(approach_yaw - current_yaw);
    const double yaw_rate =
      std::clamp(gain_yaw_ * yaw_error, -goal.max_yaw_rate, goal.max_yaw_rate);
    const double depth_error = target_position.z - current_position.z;
    const double vertical_speed =
      std::clamp(gain_z_ * depth_error, -goal.max_vertical_speed, goal.max_vertical_speed);

    double forward_speed = goal.max_forward_speed;
    double lateral_speed = 0.0;
    if (distance_to_goal < slowdown_distance) {
      forward_speed = goal.max_forward_speed * distance_to_goal / slowdown_distance;
    }

    if (use_holonomic_control) {
      const double cos_yaw = std::cos(current_yaw);
      const double sin_yaw = std::sin(current_yaw);
      const double body_error_x = cos_yaw * error_x + sin_yaw * error_y;
      const double body_error_y = -sin_yaw * error_x + cos_yaw * error_y;
      forward_speed = std::clamp(
        gain_x_ * body_error_x,
        -goal.max_forward_speed,
        goal.max_forward_speed);
      lateral_speed = std::clamp(
        gain_y_ * body_error_y,
        -goal.max_forward_speed,
        goal.max_forward_speed);
    } else {
      const double yaw_alignment_scale =
        std::clamp(1.0 - std::abs(yaw_error) / misalignment_slowdown_yaw, 0.0, 1.0);
      forward_speed *= yaw_alignment_scale;
    }

    publishBodyVelocity(forward_speed, lateral_speed, vertical_speed, yaw_rate);
    publishDepthSetpoint(target_position.z);
    publishPlannedPose();

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
      distance_to_goal < goal.position_tolerance &&
      std::abs(depth_error) < depth_tolerance &&
      std::abs(yaw_error) < goal.yaw_tolerance)
    {
      publishZeroVelocity();
      result->success = true;
      result->message = "GoToPose target reached";
      goal_handle->succeed(result);
      finish();
      return;
    }

    if (elapsed > timeout_duration) {
      publishZeroVelocity();
      result->success = false;
      result->message = "GoToPose goal timed out";
      goal_handle->abort(result);
      finish();
      return;
    }

    loop_rate.sleep();
  }

  publishZeroVelocity();
  result->success = false;
  result->message = "ROS shutdown while executing GoToPose goal";
  goal_handle->abort(result);
  finish();
}

void GoToPoseLifecycleActionServer::handleMarkerFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (!isInactive()) {
    return;
  }
  if (!feedback || feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    return;
  }
  if (!isFinitePose(feedback->pose) || !isValidQuaternion(feedback->pose.orientation)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(planned_pose_mutex_);
    planned_pose_.header.frame_id = frame_id_;
    planned_pose_.header.stamp = this->now();
    planned_pose_.pose = feedback->pose;
    has_planned_pose_ = true;
  }

  publishPlannedPose();
}

void GoToPoseLifecycleActionServer::rebuildInteractiveMarker()
{
  if (!interactive_marker_server_) {
    return;
  }

  PoseStampedMsg marker_pose;
  {
    std::lock_guard<std::mutex> lock(planned_pose_mutex_);
    marker_pose = planned_pose_;
  }

  interactive_marker_server_->clear();

  visualization_msgs::msg::InteractiveMarker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->now();
  marker.name = "go_to_pose_goal";
  marker.description = "GoToPose goal";
  marker.scale = 0.7;
  marker.pose = marker_pose.pose;
  if (!isValidQuaternion(marker.pose.orientation)) {
    marker.pose.orientation.w = 1.0;
  }

  visualization_msgs::msg::Marker sphere;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.scale.x = 0.22;
  sphere.scale.y = 0.22;
  sphere.scale.z = 0.22;
  sphere.color.r = 0.2F;
  sphere.color.g = 0.8F;
  sphere.color.b = 1.0F;
  sphere.color.a = 1.0F;

  visualization_msgs::msg::InteractiveMarkerControl sphere_control;
  sphere_control.always_visible = true;
  sphere_control.markers.push_back(sphere);
  marker.controls.push_back(sphere_control);

  visualization_msgs::msg::Marker yaw_arrow;
  yaw_arrow.type = visualization_msgs::msg::Marker::ARROW;
  yaw_arrow.scale.x = 0.45;
  yaw_arrow.scale.y = 0.08;
  yaw_arrow.scale.z = 0.08;
  yaw_arrow.color.r = 1.0F;
  yaw_arrow.color.g = 0.7F;
  yaw_arrow.color.b = 0.1F;
  yaw_arrow.color.a = 1.0F;

  visualization_msgs::msg::InteractiveMarkerControl yaw_visual_control;
  yaw_visual_control.always_visible = true;
  yaw_visual_control.markers.push_back(yaw_arrow);
  marker.controls.push_back(yaw_visual_control);

  visualization_msgs::msg::InteractiveMarkerControl move_x_control;
  move_x_control.name = "move_x";
  move_x_control.orientation.w = 1.0;
  move_x_control.orientation.x = 1.0;
  move_x_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  move_x_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(move_x_control);

  visualization_msgs::msg::InteractiveMarkerControl move_y_control;
  move_y_control.name = "move_y";
  move_y_control.orientation.w = 1.0;
  move_y_control.orientation.z = 1.0;
  move_y_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  move_y_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(move_y_control);

  visualization_msgs::msg::InteractiveMarkerControl move_z_control;
  move_z_control.name = "move_z";
  move_z_control.orientation.w = 1.0;
  move_z_control.orientation.y = 1.0;
  move_z_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  move_z_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(move_z_control);

  visualization_msgs::msg::InteractiveMarkerControl rotate_yaw_control;
  rotate_yaw_control.name = "rotate_yaw";
  rotate_yaw_control.orientation.w = 1.0;
  rotate_yaw_control.orientation.y = 1.0;
  rotate_yaw_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  rotate_yaw_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(rotate_yaw_control);

  interactive_marker_server_->insert(
    marker,
    std::bind(&GoToPoseLifecycleActionServer::handleMarkerFeedback, this, std::placeholders::_1));
  interactive_marker_server_->applyChanges();
}

void GoToPoseLifecycleActionServer::clearInteractiveMarker()
{
  if (!interactive_marker_server_) {
    return;
  }
  interactive_marker_server_->clear();
  interactive_marker_server_->applyChanges();
}

void GoToPoseLifecycleActionServer::publishPlannedPose()
{
  if (!target_pose_pub_) {
    return;
  }

  PoseStampedMsg pose;
  {
    std::lock_guard<std::mutex> lock(planned_pose_mutex_);
    pose = planned_pose_;
  }
  pose.header.stamp = this->now();
  target_pose_pub_->publish(pose);
}

bool GoToPoseLifecycleActionServer::getGoalPose(
  const GoToPose::Goal & goal,
  PoseStampedMsg & target_pose,
  double & target_yaw,
  std::string & error_message) const
{
  if (isExplicitGoalPose(goal)) {
    target_pose = goal.target_pose;
    target_yaw = std::isfinite(goal.target_yaw) ?
      goal.target_yaw : quaternionToYaw(goal.target_pose.pose.orientation);
    return true;
  }

  std::lock_guard<std::mutex> lock(planned_pose_mutex_);
  if (!has_planned_pose_) {
    error_message = "No planned marker pose is available";
    return false;
  }

  target_pose = planned_pose_;
  target_yaw = quaternionToYaw(planned_pose_.pose.orientation);
  return true;
}

GoToPoseLifecycleActionServer::GoToPose::Goal
GoToPoseLifecycleActionServer::resolveGoalDefaults(const GoToPose::Goal & goal) const
{
  auto resolved_goal = goal;
  resolved_goal.holonomic = goal.holonomic || default_holonomic_;
  resolved_goal.max_forward_speed =
    sanitizePositive(goal.max_forward_speed, default_max_forward_speed_);
  resolved_goal.max_vertical_speed =
    sanitizePositive(goal.max_vertical_speed, default_max_vertical_speed_);
  resolved_goal.max_yaw_rate = sanitizePositive(goal.max_yaw_rate, default_max_yaw_rate_);
  resolved_goal.position_tolerance =
    sanitizePositive(goal.position_tolerance, default_position_tolerance_);
  resolved_goal.yaw_tolerance = sanitizePositive(goal.yaw_tolerance, default_yaw_tolerance_);
  resolved_goal.slowdown_distance =
    goal.slowdown_distance > 0.0 ? goal.slowdown_distance : default_slowdown_distance_;
  resolved_goal.timeout = sanitizePositive(goal.timeout, default_timeout_);
  return resolved_goal;
}

bool GoToPoseLifecycleActionServer::isExplicitGoalPose(const GoToPose::Goal & goal) const
{
  return !goal.target_pose.header.frame_id.empty() &&
         isFinitePose(goal.target_pose.pose) &&
         isValidQuaternion(goal.target_pose.pose.orientation);
}

bool GoToPoseLifecycleActionServer::isInactive()
{
  return this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

bool GoToPoseLifecycleActionServer::isActive()
{
  return this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool GoToPoseLifecycleActionServer::isExecuting() const
{
  return executing_.load();
}

bool GoToPoseLifecycleActionServer::tryStartExecution()
{
  bool expected = false;
  if (!executing_.compare_exchange_strong(expected, true)) {
    return false;
  }
  stop_requested_.store(false);
  return true;
}

void GoToPoseLifecycleActionServer::finishExecution()
{
  executing_.store(false);
  stop_requested_.store(false);
}

void GoToPoseLifecycleActionServer::scheduleDeactivateAfterGoal()
{
  std::thread{
    [this]()
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (!isActive()) {
        return;
      }

      RCLCPP_INFO(get_logger(), "GoToPose goal finished: deactivating lifecycle node");
      try {
        this->deactivate();
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to deactivate GoToPose lifecycle node after goal completion: %s",
          exception.what());
      }
    }}.detach();
}

void GoToPoseLifecycleActionServer::requestStop()
{
  stop_requested_.store(true);
}

void GoToPoseLifecycleActionServer::joinExecutionThread()
{
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
}

void GoToPoseLifecycleActionServer::cleanupResources()
{
  requestStop();
  joinExecutionThread();
  publishZeroVelocity();
  clearInteractiveMarker();
  action_server_.reset();
  navigator_sub_.reset();
  target_pose_pub_.reset();
  body_velocity_pub_.reset();
  depth_setpoint_pub_.reset();
  set_control_mode_client_.reset();
  interactive_marker_server_.reset();
  {
    std::lock_guard<std::mutex> lock(navigator_mutex_);
    last_navigator_msg_.reset();
  }
  finishExecution();
}

bool GoToPoseLifecycleActionServer::requestControlMode(
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

void GoToPoseLifecycleActionServer::requestHoldPosition(const std::string & reason)
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

void GoToPoseLifecycleActionServer::publishZeroVelocity()
{
  publishBodyVelocity(0.0, 0.0, 0.0, 0.0);
}

void GoToPoseLifecycleActionServer::publishBodyVelocity(
  double forward_speed,
  double lateral_speed,
  double vertical_speed,
  double yaw_rate)
{
  if (!body_velocity_pub_) {
    return;
  }

  TwistMsg msg;
  msg.linear.x = forward_speed;
  msg.linear.y = lateral_speed;
  msg.linear.z = vertical_speed;
  msg.angular.z = yaw_rate;
  body_velocity_pub_->publish(msg);
}

void GoToPoseLifecycleActionServer::publishDepthSetpoint(double target_depth)
{
  if (!depth_setpoint_pub_) {
    return;
  }

  DepthSetPointMsg msg;
  msg.position.x = std::numeric_limits<double>::quiet_NaN();
  msg.position.y = std::numeric_limits<double>::quiet_NaN();
  msg.position.z = target_depth;
  msg.rpy.x = std::numeric_limits<double>::quiet_NaN();
  msg.rpy.y = std::numeric_limits<double>::quiet_NaN();
  msg.rpy.z = std::numeric_limits<double>::quiet_NaN();
  depth_setpoint_pub_->publish(msg);
}

bool GoToPoseLifecycleActionServer::getNavigatorSnapshot(
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

std::string GoToPoseLifecycleActionServer::namespacedTopic(const std::string & suffix) const
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
  auto node = std::make_shared<sura_actions::GoToPoseLifecycleActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
