#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sura_actions/action/go_to_pose_non_holonomic.hpp"
#include "sura_actions/srv/set_control_mode.hpp"
#include "sura_msgs/msg/auv_controller_set_point.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace sura_actions
{

class GoToPoseNonHolonomicActionServer : public rclcpp::Node
{
public:
  using GoToPoseNonHolonomic = sura_actions::action::GoToPoseNonHolonomic;
  using GoalHandleGoToPoseNonHolonomic =
    rclcpp_action::ServerGoalHandle<GoToPoseNonHolonomic>;
  using SetControlMode = sura_actions::srv::SetControlMode;
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
  using TwistMsg = geometry_msgs::msg::Twist;
  using DepthSetPointMsg = sura_msgs::msg::AuvControllerSetPoint;

  explicit GoToPoseNonHolonomicActionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoToPoseNonHolonomic::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleGoToPoseNonHolonomic> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleGoToPoseNonHolonomic> goal_handle);
  void execute(const std::shared_ptr<GoalHandleGoToPoseNonHolonomic> goal_handle);

  bool requestControlMode(uint8_t mode, const std::string & reason, std::string & error_message);
  void requestHoldPosition(const std::string & reason);
  void publishZeroVelocity();
  void publishBodyVelocity(
    double forward_speed,
    double lateral_speed,
    double vertical_speed,
    double yaw_rate);
  void publishDepthSetpoint(double target_depth);
  void publishTargetPose(const GoToPoseNonHolonomic::Goal & goal);
  bool getNavigatorSnapshot(NavigatorMsg & navigator_msg, rclcpp::Time & stamp) const;
  std::string namespacedTopic(const std::string & suffix) const;

  rclcpp_action::Server<GoToPoseNonHolonomic>::SharedPtr action_server_;
  rclcpp::Client<SetControlMode>::SharedPtr set_control_mode_client_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr body_velocity_pub_;
  rclcpp::Publisher<DepthSetPointMsg>::SharedPtr depth_setpoint_pub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;

  mutable std::mutex navigator_mutex_;
  NavigatorMsg::SharedPtr last_navigator_msg_;
  rclcpp::Time last_navigator_time_;

  std::string robot_namespace_;
  std::string action_name_;
  std::string set_control_mode_service_;
  std::string navigator_topic_;
  std::string body_velocity_command_topic_;
  std::string depth_setpoint_topic_;
  std::string target_pose_topic_;
  double default_depth_tolerance_{0.1};
  double navigator_timeout_{2.0};
  double mode_request_timeout_{5.0};
  double control_loop_rate_{15.0};
  double gain_x_{0.4};
  double gain_y_{0.4};
  double gain_z_{0.5};
  double gain_yaw_{1.0};
  double misalignment_slowdown_yaw_{1.0};
};

}  // namespace sura_actions
