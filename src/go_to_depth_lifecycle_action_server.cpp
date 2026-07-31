#include "sura_actions/go_to_depth_lifecycle_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"

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

std::string modeName(uint8_t mode)
{
  switch (mode) {
    case GoToDepthLifecycleActionServer::SetControlMode::Request::MANUAL:
      return "MANUAL";
    default:
      return std::to_string(mode);
  }
}
}  // namespace

GoToDepthLifecycleActionServer::GoToDepthLifecycleActionServer(
  const rclcpp::NodeOptions & options)
: LifecycleNode("go_to_depth_lifecycle_action_node", options)
{
}

GoToDepthLifecycleActionServer::~GoToDepthLifecycleActionServer()
{
  cleanupResources();
}

GoToDepthLifecycleActionServer::CallbackReturn GoToDepthLifecycleActionServer::on_configure(
  const rclcpp_lifecycle::State &)
{
  robot_namespace_ = getOrDeclareParameter<std::string>(*this, "robot_namespace", "sura");
  action_name_ = getOrDeclareParameter<std::string>(
    *this, "action_name", namespacedTopic("actions/go_to_depth"));
  set_control_mode_service_ = getOrDeclareParameter<std::string>(
    *this, "set_control_mode_service", namespacedTopic("control_manager/set_mode"));
  navigator_topic_ = getOrDeclareParameter<std::string>(
    *this, "navigator_topic", namespacedTopic("navigator/navigation"));
  depth_setpoint_topic_ = getOrDeclareParameter<std::string>(
    *this, "depth_setpoint_topic", namespacedTopic("controller/depth_hold/set_point"));
  default_depth_tolerance_ =
    getOrDeclareParameter<double>(*this, "default_depth_tolerance", 0.10);
  default_timeout_ = getOrDeclareParameter<double>(*this, "default_timeout", 30.0);
  navigator_timeout_ = getOrDeclareParameter<double>(*this, "navigator_timeout", 2.0);
  mode_request_timeout_ = getOrDeclareParameter<double>(*this, "mode_request_timeout", 5.0);
  setpoint_publish_rate_ = getOrDeclareParameter<double>(*this, "setpoint_publish_rate", 10.0);

  set_control_mode_client_ = this->create_client<SetControlMode>(set_control_mode_service_);
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

  action_server_ = rclcpp_action::create_server<GoToDepth>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    action_name_,
    std::bind(
      &GoToDepthLifecycleActionServer::handleGoal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&GoToDepthLifecycleActionServer::handleCancel, this, std::placeholders::_1),
    std::bind(&GoToDepthLifecycleActionServer::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "GoToDepthLifecycleActionServer configured");
  RCLCPP_INFO(get_logger(), "Using action '%s'", action_name_.c_str());
  RCLCPP_INFO(get_logger(), "Using set mode service '%s'", set_control_mode_service_.c_str());
  RCLCPP_INFO(get_logger(), "Using navigator topic '%s'", navigator_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Using depth setpoint topic '%s'", depth_setpoint_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

GoToDepthLifecycleActionServer::CallbackReturn GoToDepthLifecycleActionServer::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "GoToDepthLifecycleActionServer active: goals are accepted");
  return CallbackReturn::SUCCESS;
}

GoToDepthLifecycleActionServer::CallbackReturn GoToDepthLifecycleActionServer::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  requestStop();
  joinExecutionThread();
  RCLCPP_INFO(get_logger(), "GoToDepthLifecycleActionServer inactive");
  return CallbackReturn::SUCCESS;
}

GoToDepthLifecycleActionServer::CallbackReturn GoToDepthLifecycleActionServer::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

GoToDepthLifecycleActionServer::CallbackReturn GoToDepthLifecycleActionServer::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse GoToDepthLifecycleActionServer::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const GoToDepth::Goal> goal)
{
  const double timeout = sanitizePositive(goal->timeout, default_timeout_);
  const double tolerance = sanitizePositive(goal->depth_tolerance, default_depth_tolerance_);

  if (!isActive()) {
    RCLCPP_WARN(get_logger(), "Rejecting GoToDepth goal because the node is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (isExecuting()) {
    RCLCPP_WARN(get_logger(), "Rejecting GoToDepth goal because another goal is executing");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!std::isfinite(goal->target_depth) || timeout <= 0.0 || tolerance <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Rejecting GoToDepth goal because target_depth must be finite and timeout/tolerance positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted GoToDepth goal: target_depth=%.3f tolerance=%.3f timeout=%.3f",
    goal->target_depth,
    tolerance,
    timeout);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GoToDepthLifecycleActionServer::handleCancel(
  const std::shared_ptr<GoalHandleGoToDepth>)
{
  RCLCPP_INFO(get_logger(), "Canceling GoToDepth goal");
  requestStop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GoToDepthLifecycleActionServer::handleAccepted(
  const std::shared_ptr<GoalHandleGoToDepth> goal_handle)
{
  if (!tryStartExecution()) {
    auto result = std::make_shared<GoToDepth::Result>();
    result->success = false;
    result->message = "Another GoToDepth goal is already executing";
    result->final_depth = std::numeric_limits<double>::quiet_NaN();
    goal_handle->abort(result);
    return;
  }

  std::lock_guard<std::mutex> lock(execution_mutex_);
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
  execution_thread_ = std::thread{
    std::bind(&GoToDepthLifecycleActionServer::execute, this, std::placeholders::_1),
    goal_handle};
}

void GoToDepthLifecycleActionServer::execute(
  const std::shared_ptr<GoalHandleGoToDepth> goal_handle)
{
  const auto finish = [this]() { finishExecution(); };
  const auto goal = goal_handle->get_goal();
  const double target_depth = goal->target_depth;
  const double depth_tolerance = sanitizePositive(goal->depth_tolerance, default_depth_tolerance_);
  const double timeout = sanitizePositive(goal->timeout, default_timeout_);
  const auto start_time = this->now();
  const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(timeout);
  const rclcpp::Duration navigator_timeout_duration =
    rclcpp::Duration::from_seconds(sanitizePositive(navigator_timeout_, 2.0));
  const double publish_rate = sanitizePositive(setpoint_publish_rate_, 10.0);

  auto result = std::make_shared<GoToDepth::Result>();
  auto feedback = std::make_shared<GoToDepth::Feedback>();

  std::string mode_error;
  if (!requestControlMode(
      SetControlMode::Request::MANUAL,
      "GoToDepth action goal accepted",
      mode_error))
  {
    double current_depth = std::numeric_limits<double>::quiet_NaN();
    getCurrentDepth(current_depth);
    result->success = false;
    result->message = "Failed to request MANUAL mode: " + mode_error;
    result->final_depth = current_depth;
    goal_handle->abort(result);
    finish();
    return;
  }

  rclcpp::Rate loop_rate(publish_rate);

  while (rclcpp::ok() && !stop_requested_.load()) {
    if (goal_handle->is_canceling()) {
      double current_depth = std::numeric_limits<double>::quiet_NaN();
      getCurrentDepth(current_depth);
      requestTerminalMode("GoToDepth action canceled");
      result->success = false;
      result->message = "GoToDepth goal canceled";
      result->final_depth = current_depth;
      goal_handle->canceled(result);
      finish();
      return;
    }

    double current_depth = std::numeric_limits<double>::quiet_NaN();
    const bool has_depth = getCurrentDepth(current_depth);
    const double depth_error = has_depth ? target_depth - current_depth :
      std::numeric_limits<double>::quiet_NaN();

    publishDepthSetpoint(target_depth);

    feedback->current_depth = current_depth;
    feedback->depth_error = depth_error;
    feedback->time_remaining = std::max(
      0.0,
      timeout - (this->now() - start_time).seconds());
    feedback->state = has_depth ? "going_to_depth" : "waiting_for_navigation";
    goal_handle->publish_feedback(feedback);

    if (has_depth && std::abs(depth_error) <= depth_tolerance) {
      requestTerminalMode("GoToDepth action succeeded");
      result->success = true;
      result->message = "GoToDepth target reached";
      result->final_depth = current_depth;
      goal_handle->succeed(result);
      finish();
      return;
    }

    const auto elapsed = this->now() - start_time;
    if (elapsed > timeout_duration) {
      requestTerminalMode("GoToDepth action timed out");
      result->success = false;
      result->message = "GoToDepth goal timed out";
      result->final_depth = current_depth;
      goal_handle->abort(result);
      finish();
      return;
    }

    {
      std::lock_guard<std::mutex> lock(navigator_mutex_);
      const bool navigator_missing =
        !last_navigator_msg_ && elapsed > navigator_timeout_duration;
      const bool navigator_stale =
        last_navigator_msg_ && (this->now() - last_navigator_time_) > navigator_timeout_duration;
      if (navigator_missing || navigator_stale) {
        requestTerminalMode("GoToDepth action lost navigation");
        result->success = false;
        result->message = navigator_missing ? "Navigator data was not received" :
          "Navigator data timed out";
        result->final_depth = current_depth;
        goal_handle->abort(result);
        finish();
        return;
      }
    }

    loop_rate.sleep();
  }

  double current_depth = std::numeric_limits<double>::quiet_NaN();
  getCurrentDepth(current_depth);
  requestTerminalMode("GoToDepth action stopped");
  result->success = false;
  result->message = rclcpp::ok() ? "GoToDepth goal stopped" :
    "ROS shutdown while executing GoToDepth goal";
  result->final_depth = current_depth;
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else {
    goal_handle->abort(result);
  }
  finish();
}

bool GoToDepthLifecycleActionServer::isActive()
{
  return this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool GoToDepthLifecycleActionServer::isExecuting() const
{
  return executing_.load();
}

bool GoToDepthLifecycleActionServer::tryStartExecution()
{
  bool expected = false;
  if (!executing_.compare_exchange_strong(expected, true)) {
    return false;
  }
  stop_requested_.store(false);
  return true;
}

void GoToDepthLifecycleActionServer::finishExecution()
{
  executing_.store(false);
  stop_requested_.store(false);
}

void GoToDepthLifecycleActionServer::requestStop()
{
  stop_requested_.store(true);
}

void GoToDepthLifecycleActionServer::joinExecutionThread()
{
  std::lock_guard<std::mutex> lock(execution_mutex_);
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
}

void GoToDepthLifecycleActionServer::cleanupResources()
{
  requestStop();
  joinExecutionThread();
  action_server_.reset();
  navigator_sub_.reset();
  depth_setpoint_pub_.reset();
  set_control_mode_client_.reset();
  {
    std::lock_guard<std::mutex> lock(navigator_mutex_);
    last_navigator_msg_.reset();
  }
  finishExecution();
}

bool GoToDepthLifecycleActionServer::requestControlMode(
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

void GoToDepthLifecycleActionServer::requestTerminalMode(const std::string & reason)
{
  std::string mode_error;
  if (!requestControlMode(SetControlMode::Request::MANUAL, reason, mode_error)) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to request terminal MANUAL mode: %s",
      mode_error.c_str());
  }
}

void GoToDepthLifecycleActionServer::publishDepthSetpoint(double target_depth)
{
  DepthSetPointMsg msg;
  msg.position.z = target_depth;
  depth_setpoint_pub_->publish(msg);
}

bool GoToDepthLifecycleActionServer::getCurrentDepth(double & depth) const
{
  std::lock_guard<std::mutex> lock(navigator_mutex_);
  if (!last_navigator_msg_) {
    return false;
  }

  depth = last_navigator_msg_->position.position.z;
  return std::isfinite(depth);
}

std::string GoToDepthLifecycleActionServer::namespacedTopic(const std::string & suffix) const
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
  auto node = std::make_shared<sura_actions::GoToDepthLifecycleActionServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
