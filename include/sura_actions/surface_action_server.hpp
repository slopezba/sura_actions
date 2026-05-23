#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "sura_actions/action/surface.hpp"
#include "sura_actions/srv/set_control_mode.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace sura_actions
{

class SurfaceActionServer : public rclcpp::Node
{
public:
  using Surface = sura_actions::action::Surface;
  using GoalHandleSurface = rclcpp_action::ServerGoalHandle<Surface>;
  using SetControlMode = sura_actions::srv::SetControlMode;
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using WrenchMsg = geometry_msgs::msg::Wrench;

  explicit SurfaceActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Surface::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleSurface> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleSurface> goal_handle);
  void execute(const std::shared_ptr<GoalHandleSurface> goal_handle);

  bool requestControlMode(uint8_t mode, const std::string & reason, std::string & error_message);
  void requestTerminalMode(const std::string & reason);
  void publishSurfaceWrench(double surface_force_z);
  bool getCurrentDepth(double & depth) const;
  std::string namespacedTopic(const std::string & suffix) const;

  rclcpp_action::Server<Surface>::SharedPtr action_server_;
  rclcpp::Client<SetControlMode>::SharedPtr set_control_mode_client_;
  rclcpp::Publisher<WrenchMsg>::SharedPtr surface_wrench_pub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;

  mutable std::mutex navigator_mutex_;
  NavigatorMsg::SharedPtr last_navigator_msg_;
  rclcpp::Time last_navigator_time_;

  std::string robot_namespace_;
  std::string action_name_;
  std::string set_control_mode_service_;
  std::string navigator_topic_;
  std::string depth_feedforward_topic_;
  double default_depth_tolerance_{0.1};
  double default_timeout_{30.0};
  double navigator_timeout_{2.0};
  double mode_request_timeout_{5.0};
  double setpoint_publish_rate_{10.0};
};

}  // namespace sura_actions
