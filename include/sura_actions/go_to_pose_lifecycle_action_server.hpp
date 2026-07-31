#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sura_actions/action/go_to_pose.hpp"
#include "sura_actions/srv/set_control_mode.hpp"
#include "sura_msgs/msg/auv_controller_set_point.hpp"
#include "sura_msgs/msg/navigator.hpp"
#include "sura_msgs/msg/sura_velocity_command.hpp"
#include "sura_msgs/srv/clear_controller_intents.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

namespace sura_actions
{

class GoToPoseLifecycleActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using GoToPose = sura_actions::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;
  using SetControlMode = sura_actions::srv::SetControlMode;
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
  using SuraVelocityCommandMsg = sura_msgs::msg::SuraVelocityCommand;
  using TwistMsg = geometry_msgs::msg::Twist;
  using DepthSetPointMsg = sura_msgs::msg::AuvControllerSetPoint;
  using InteractiveMarkerServer = interactive_markers::InteractiveMarkerServer;

  explicit GoToPoseLifecycleActionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GoToPoseLifecycleActionServer() override;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoToPose::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle);

  void handleMarkerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);
  void rebuildInteractiveMarker();
  void clearInteractiveMarker();
  void publishPlannedPose();

  bool getGoalPose(
    const GoToPose::Goal & goal,
    PoseStampedMsg & target_pose,
    double & target_yaw,
    std::string & error_message) const;
  GoToPose::Goal resolveGoalDefaults(const GoToPose::Goal & goal) const;
  bool isExplicitGoalPose(const GoToPose::Goal & goal) const;
  bool isInactive();
  bool isActive();
  bool isExecuting() const;
  bool tryStartExecution();
  void finishExecution();
  void scheduleDeactivateAfterGoal();
  void requestStop();
  void joinExecutionThread();
  void cleanupResources();

  bool requestControlMode(uint8_t mode, const std::string & reason, std::string & error_message);
  void requestHoldPosition(const std::string & reason);
  void publishZeroVelocity();
  void publishBodyVelocity(
    double forward_speed,
    double lateral_speed,
    double vertical_speed,
    double yaw_rate);
  void publishDepthSetpoint(double target_depth);
  void clearBodyVelocityIntents();
  void clearControllerIntent(const std::string & controller_name);
  bool getNavigatorSnapshot(NavigatorMsg & navigator_msg, rclcpp::Time & stamp) const;
  std::string namespacedTopic(const std::string & suffix) const;

  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
  rclcpp::Client<SetControlMode>::SharedPtr set_control_mode_client_;
  rclcpp::Client<sura_msgs::srv::ClearControllerIntents>::SharedPtr clear_intents_client_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<SuraVelocityCommandMsg>::SharedPtr body_velocity_pub_;
  rclcpp::Publisher<DepthSetPointMsg>::SharedPtr depth_setpoint_pub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  std::unique_ptr<InteractiveMarkerServer> interactive_marker_server_;

  mutable std::mutex navigator_mutex_;
  NavigatorMsg::SharedPtr last_navigator_msg_;
  rclcpp::Time last_navigator_time_;

  mutable std::mutex planned_pose_mutex_;
  PoseStampedMsg planned_pose_;
  bool has_planned_pose_{false};

  mutable std::mutex execution_mutex_;
  std::thread execution_thread_;
  std::atomic_bool executing_{false};
  std::atomic_bool stop_requested_{false};

  std::string robot_namespace_;
  std::string frame_id_;
  std::string action_name_;
  std::string set_control_mode_service_;
  std::string navigator_topic_;
  std::string arbitrator_velocity_topic_;
  std::string clear_controller_intents_service_;
  std::string body_velocity_controller_name_{"body_velocity"};
  std::string requester_{"go_to_pose"};
  int priority_{60};
  std::string depth_setpoint_topic_;
  std::string target_pose_topic_;
  std::string interactive_marker_namespace_;
  bool default_holonomic_{false};
  double default_max_forward_speed_{0.3};
  double default_max_vertical_speed_{0.3};
  double default_max_yaw_rate_{0.5};
  double default_position_tolerance_{0.2};
  double default_yaw_tolerance_{0.2};
  double default_slowdown_distance_{0.5};
  double default_timeout_{60.0};
  double default_depth_tolerance_{0.1};
  double navigator_timeout_{2.0};
  double mode_request_timeout_{5.0};
  double control_loop_rate_{15.0};
  double gain_x_{0.4};
  double gain_y_{0.4};
  double gain_z_{1.0};
  double gain_yaw_{1.0};
  double misalignment_slowdown_yaw_{1.0};
  bool hold_after_reaching_{true};
};

}  // namespace sura_actions
