#include "sura_actions/sura_control_manager.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace sura_actions
{
namespace
{
double sanitizePositive(double value, double fallback)
{
  return value > 0.0 ? value : fallback;
}
}  // namespace

SuraControlManager::SuraControlManager(const rclcpp::NodeOptions & options)
: Node("sura_control_manager_node", options)
{
  robot_namespace_ = this->declare_parameter<std::string>("robot_namespace", "sura");
  controller_manager_ = this->declare_parameter<std::string>(
    "controller_manager", namespacedTopic("controller/controller_manager"));
  switch_service_name_ = this->declare_parameter<std::string>(
    "switch_service_name", "switch_controller");
  switch_timeout_ = this->declare_parameter<double>("switch_timeout", 5.0);
  strictness_ = this->declare_parameter<int>(
    "strictness", SwitchController::Request::BEST_EFFORT);

  loadModeConfigs();

  set_mode_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  switch_client_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  switch_controller_client_ = this->create_client<SwitchController>(
    controller_manager_ + "/" + switch_service_name_,
    rmw_qos_profile_services_default,
    switch_client_callback_group_);

  set_mode_srv_ = this->create_service<SetControlMode>(
    namespacedTopic("control_manager/set_mode"),
    std::bind(
      &SuraControlManager::handleSetMode,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default,
    set_mode_callback_group_);

  RCLCPP_INFO(
    get_logger(),
    "SURA control manager ready on '%s'",
    namespacedTopic("control_manager/set_mode").c_str());
  RCLCPP_INFO(
    get_logger(),
    "Using controller manager service '%s/%s'",
    controller_manager_.c_str(),
    switch_service_name_.c_str());
}

void SuraControlManager::handleSetMode(
  const std::shared_ptr<SetControlMode::Request> request,
  std::shared_ptr<SetControlMode::Response> response)
{
  const auto mode_it = modes_.find(request->mode);
  if (mode_it == modes_.end()) {
    response->success = false;
    response->message = "Unsupported control mode: " + std::to_string(request->mode);
    response->active_mode = active_mode_;
    return;
  }

  std::string error_message;
  if (!switchToMode(mode_it->second, error_message)) {
    response->success = false;
    response->message = "Failed to switch to " + mode_it->second.name + ": " + error_message;
    response->active_mode = active_mode_;
    return;
  }

  active_mode_ = request->mode;
  response->success = true;
  response->message = "Control mode set to " + mode_it->second.name;
  response->active_mode = active_mode_;

  RCLCPP_INFO(
    get_logger(),
    "Control mode set to %s. Reason: %s",
    mode_it->second.name.c_str(),
    request->reason.c_str());
}

bool SuraControlManager::switchToMode(
  const ModeConfig & mode_config,
  std::string & error_message)
{
  const double timeout = sanitizePositive(switch_timeout_, 5.0);
  if (!switch_controller_client_->wait_for_service(std::chrono::duration<double>(timeout))) {
    error_message = "switch_controller service is not available";
    return false;
  }

  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = mode_config.activate_controllers;
  request->deactivate_controllers = mode_config.deactivate_controllers;
  request->strictness = strictness_;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration::from_seconds(timeout);

  auto future = switch_controller_client_->async_send_request(request);
  const auto status = future.wait_for(std::chrono::duration<double>(timeout + 1.0));
  if (status != std::future_status::ready) {
    error_message = "switch_controller request timed out";
    return false;
  }

  const auto response = future.get();
  if (!response->ok) {
    error_message = "controller_manager rejected the switch";
    return false;
  }

  return true;
}

void SuraControlManager::loadModeConfigs()
{
  const auto load_mode =
    [this](uint8_t mode, const std::string & key, std::vector<std::string> activate,
      std::vector<std::string> deactivate)
    {
      const auto activate_param = "modes." + key + ".activate";
      const auto deactivate_param = "modes." + key + ".deactivate";
      modes_[mode] = ModeConfig{
        modeName(mode),
        this->declare_parameter<std::vector<std::string>>(activate_param, activate),
        this->declare_parameter<std::vector<std::string>>(deactivate_param, deactivate)};
    };

  load_mode(
    SetControlMode::Request::MANUAL,
    "manual",
    {"body_force", "depth_hold"},
    {});
  load_mode(
    SetControlMode::Request::FOLLOW_PATH,
    "follow_path",
    {"body_force", "body_velocity", "depth_hold"},
    {"position_hold", "stabilize", "mpc_4dof", "thruster_test_controller"});
  load_mode(
    SetControlMode::Request::SURFACE,
    "surface",
    {"body_force", "depth_hold"},
    {"position_hold", "body_velocity", "stabilize", "mpc_4dof", "thruster_test_controller"});
  load_mode(
    SetControlMode::Request::HOLD_POSITION,
    "hold_position",
    {"body_force", "body_velocity", "position_hold"},
    {"stabilize", "depth_hold", "mpc_4dof", "thruster_test_controller"});
  load_mode(
    SetControlMode::Request::BODY_VELOCITY,
    "body_velocity",
    {"body_force", "body_velocity", "depth_hold"},
    {"position_hold", "stabilize", "mpc_4dof", "thruster_test_controller"});
  load_mode(
    SetControlMode::Request::EMERGENCY_STOP,
    "emergency_stop",
    {},
    {"position_hold", "body_velocity", "stabilize", "depth_hold", "mpc_4dof",
      "thruster_test_controller"});
}

std::string SuraControlManager::namespacedTopic(const std::string & suffix) const
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

std::string SuraControlManager::modeName(uint8_t mode)
{
  switch (mode) {
    case SetControlMode::Request::MANUAL:
      return "MANUAL";
    case SetControlMode::Request::FOLLOW_PATH:
      return "FOLLOW_PATH";
    case SetControlMode::Request::SURFACE:
      return "SURFACE";
    case SetControlMode::Request::HOLD_POSITION:
      return "HOLD_POSITION";
    case SetControlMode::Request::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case SetControlMode::Request::BODY_VELOCITY:
      return "BODY_VELOCITY";
    default:
      return "UNKNOWN";
  }
}

}  // namespace sura_actions

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sura_actions::SuraControlManager>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
