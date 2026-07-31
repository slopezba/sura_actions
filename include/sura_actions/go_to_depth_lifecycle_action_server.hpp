#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sura_actions/action/go_to_depth.hpp"
#include "sura_actions/srv/set_control_mode.hpp"
#include "sura_msgs/msg/auv_controller_set_point.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace sura_actions
{

class GoToDepthLifecycleActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using GoToDepth = sura_actions::action::GoToDepth;
  using GoalHandleGoToDepth = rclcpp_action::ServerGoalHandle<GoToDepth>;
  using SetControlMode = sura_actions::srv::SetControlMode;
  using DepthSetPointMsg = sura_msgs::msg::AuvControllerSetPoint;
  using NavigatorMsg = sura_msgs::msg::Navigator;

  explicit GoToDepthLifecycleActionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GoToDepthLifecycleActionServer() override;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoToDepth::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleGoToDepth> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleGoToDepth> goal_handle);
  void execute(const std::shared_ptr<GoalHandleGoToDepth> goal_handle);

  bool isActive();
  bool isExecuting() const;
  bool tryStartExecution();
  void finishExecution();
  void requestStop();
  void joinExecutionThread();
  void cleanupResources();

  bool requestControlMode(uint8_t mode, const std::string & reason, std::string & error_message);
  void requestTerminalMode(const std::string & reason);
  void publishDepthSetpoint(double target_depth);
  bool getCurrentDepth(double & depth) const;
  std::string namespacedTopic(const std::string & suffix) const;

  rclcpp_action::Server<GoToDepth>::SharedPtr action_server_;
  rclcpp::Client<SetControlMode>::SharedPtr set_control_mode_client_;
  rclcpp::Publisher<DepthSetPointMsg>::SharedPtr depth_setpoint_pub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;

  mutable std::mutex navigator_mutex_;
  NavigatorMsg::SharedPtr last_navigator_msg_;
  rclcpp::Time last_navigator_time_;

  mutable std::mutex execution_mutex_;
  std::thread execution_thread_;
  std::atomic_bool executing_{false};
  std::atomic_bool stop_requested_{false};

  std::string robot_namespace_;
  std::string action_name_;
  std::string set_control_mode_service_;
  std::string navigator_topic_;
  std::string depth_setpoint_topic_;
  double default_depth_tolerance_{0.1};
  double default_timeout_{30.0};
  double navigator_timeout_{2.0};
  double mode_request_timeout_{5.0};
  double setpoint_publish_rate_{10.0};
};

}  // namespace sura_actions
