#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sura_actions/action/follow_path.hpp"
#include "sura_actions/srv/add_waypoint.hpp"
#include "sura_actions/srv/path_file.hpp"
#include "sura_actions/srv/set_control_mode.hpp"
#include "sura_msgs/msg/auv_controller_set_point.hpp"
#include "sura_msgs/msg/navigator.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace sura_actions
{

struct PathWaypoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

class PathEditor
{
public:
  using PathMsg = nav_msgs::msg::Path;
  using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;
  using InteractiveMarkerServer = interactive_markers::InteractiveMarkerServer;

  void configure(
    rclcpp_lifecycle::LifecycleNode * node,
    const std::string & frame_id,
    const std::string & path_topic,
    const std::string & markers_topic,
    const std::string & interactive_marker_namespace,
    std::function<bool()> planning_enabled_cb);
  void cleanup();

  bool addWaypoint(const PathWaypoint & waypoint, int32_t & index, std::string & message);
  bool removeLastWaypoint(std::string & message);
  bool clearPath(std::string & message);
  bool savePath(const std::string & path_file, std::string & message) const;
  bool loadPath(const std::string & path_file, std::string & message);
  bool loadPathForExecution(
    const std::string & path_file,
    std::vector<PathWaypoint> & waypoints,
    std::string & frame_id,
    std::string & message) const;
  void publishPreview() const;
  void rebuildInteractiveMarkers();

private:
  void handleMarkerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);
  void publishPath() const;
  void publishMarkers() const;
  PathMsg toPathMessageLocked() const;
  MarkerArrayMsg toMarkerArrayLocked() const;
  void rebuildInteractiveMarkersLocked();

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  std::string frame_id_{"world_ned"};
  std::function<bool()> planning_enabled_cb_;
  rclcpp::Publisher<PathMsg>::SharedPtr path_pub_;
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr markers_pub_;
  std::unique_ptr<InteractiveMarkerServer> interactive_marker_server_;

  mutable std::mutex mutex_;
  std::vector<PathWaypoint> waypoints_;
};

class FollowPathExecutor
{
public:
  using FollowPath = sura_actions::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using TwistMsg = geometry_msgs::msg::Twist;
  using DepthSetPointMsg = sura_msgs::msg::AuvControllerSetPoint;
  using SetControlMode = sura_actions::srv::SetControlMode;

  void configure(
    rclcpp_lifecycle::LifecycleNode * node,
    const std::string & navigator_topic,
    const std::string & body_velocity_command_topic,
    const std::string & depth_setpoint_topic,
    const std::string & set_control_mode_service,
    double default_max_vertical_speed,
    double navigator_timeout,
    double mode_request_timeout,
    double control_loop_rate,
    double gain_x,
    double gain_y,
    double gain_z,
    double gain_yaw);
  void cleanup();

  bool isExecuting() const;
  bool tryStartExecution();
  void finishExecution();
  void requestStop();
  void joinExecutionThread();
  void startExecutionThread(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle,
    std::vector<PathWaypoint> waypoints);
  std::vector<PathWaypoint> pathFromGoal(const FollowPath::Goal & goal) const;

private:
  void execute(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle,
    std::vector<PathWaypoint> waypoints);
  bool getNavigatorSnapshot(NavigatorMsg & navigator_msg, rclcpp::Time & stamp) const;
  bool requestControlMode(uint8_t mode, const std::string & reason, std::string & error_message);
  void publishBodyVelocity(
    double forward_speed,
    double lateral_speed,
    double vertical_speed,
    double yaw_rate);
  void publishDepthSetpoint(double target_depth);
  void publishZeroVelocity();

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr body_velocity_pub_;
  rclcpp::Publisher<DepthSetPointMsg>::SharedPtr depth_setpoint_pub_;
  rclcpp::Client<SetControlMode>::SharedPtr set_control_mode_client_;

  mutable std::mutex navigator_mutex_;
  NavigatorMsg::SharedPtr last_navigator_msg_;
  rclcpp::Time last_navigator_time_;

  mutable std::mutex execution_mutex_;
  std::thread execution_thread_;
  std::atomic_bool executing_{false};
  std::atomic_bool stop_requested_{false};

  double default_max_vertical_speed_{0.3};
  double navigator_timeout_{2.0};
  double mode_request_timeout_{5.0};
  double control_loop_rate_{15.0};
  double gain_x_{0.4};
  double gain_y_{0.4};
  double gain_z_{1.0};
  double gain_yaw_{1.0};
};

class PathManagerLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using FollowPath = sura_actions::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

  explicit PathManagerLifecycleNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PathManagerLifecycleNode() override;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  bool isInactive();
  bool isActive();
  std::string namespacedTopic(const std::string & suffix) const;
  void cleanupResources();

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowPath::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleFollowPath> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle);

  void handleAddWaypoint(
    const std::shared_ptr<srv::AddWaypoint::Request> request,
    std::shared_ptr<srv::AddWaypoint::Response> response);
  void handleRemoveLastWaypoint(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleClearPath(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleSavePath(
    const std::shared_ptr<srv::PathFile::Request> request,
    std::shared_ptr<srv::PathFile::Response> response);
  void handleLoadPath(
    const std::shared_ptr<srv::PathFile::Request> request,
    std::shared_ptr<srv::PathFile::Response> response);

  std::string robot_namespace_;
  std::string frame_id_;
  std::string path_topic_;
  std::string markers_topic_;
  std::string interactive_marker_namespace_;
  std::string action_name_;
  std::string navigator_topic_;
  std::string body_velocity_command_topic_;
  std::string depth_setpoint_topic_;
  std::string set_control_mode_service_;

  PathEditor editor_;
  FollowPathExecutor executor_;
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
  rclcpp::Service<srv::AddWaypoint>::SharedPtr add_waypoint_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_last_waypoint_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_path_srv_;
  rclcpp::Service<srv::PathFile>::SharedPtr save_path_srv_;
  rclcpp::Service<srv::PathFile>::SharedPtr load_path_srv_;
};

}  // namespace sura_actions
