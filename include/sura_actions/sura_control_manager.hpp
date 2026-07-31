#pragma once

#include <map>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_actions/srv/set_control_mode.hpp"

namespace sura_actions
{

class SuraControlManager : public rclcpp::Node
{
public:
  using SetControlMode = sura_actions::srv::SetControlMode;
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  explicit SuraControlManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct ModeConfig
  {
    std::string name;
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;
  };

  void handleSetMode(
    const std::shared_ptr<SetControlMode::Request> request,
    std::shared_ptr<SetControlMode::Response> response);

  bool switchToMode(const ModeConfig & mode_config, std::string & error_message);
  bool getControllerStates(
    std::map<std::string, std::string> & controller_states,
    std::string & error_message);
  ModeConfig filterModeConfig(
    const ModeConfig & mode_config,
    const std::map<std::string, std::string> & controller_states) const;
  void loadModeConfigs();
  std::string namespacedTopic(const std::string & suffix) const;
  static std::string modeName(uint8_t mode);

  rclcpp::Service<SetControlMode>::SharedPtr set_mode_srv_;
  rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::CallbackGroup::SharedPtr set_mode_callback_group_;
  rclcpp::CallbackGroup::SharedPtr switch_client_callback_group_;

  std::map<uint8_t, ModeConfig> modes_;
  std::string robot_namespace_;
  std::string controller_manager_;
  std::string switch_service_name_;
  double switch_timeout_{5.0};
  int strictness_{SwitchController::Request::BEST_EFFORT};
  uint8_t active_mode_{SetControlMode::Request::MANUAL};
};

}  // namespace sura_actions
