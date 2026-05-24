#include "sura_actions/path_manager_lifecycle_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <future>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/create_publisher.hpp"
#include "tinyxml2.h"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"

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

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

bool isFiniteWaypoint(const PathWaypoint & waypoint)
{
  return std::isfinite(waypoint.x) &&
         std::isfinite(waypoint.y) &&
         std::isfinite(waypoint.z) &&
         std::isfinite(waypoint.yaw);
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

std::string markerName(std::size_t index)
{
  return "waypoint_" + std::to_string(index);
}

std::string modeName(uint8_t mode)
{
  switch (mode) {
    case FollowPathExecutor::SetControlMode::Request::MANUAL:
      return "MANUAL";
    case FollowPathExecutor::SetControlMode::Request::FOLLOW_PATH:
      return "FOLLOW_PATH";
    case FollowPathExecutor::SetControlMode::Request::HOLD_POSITION:
      return "HOLD_POSITION";
    case FollowPathExecutor::SetControlMode::Request::BODY_VELOCITY:
      return "BODY_VELOCITY";
    default:
      return std::to_string(mode);
  }
}

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
}  // namespace

void PathEditor::configure(
  rclcpp_lifecycle::LifecycleNode * node,
  const std::string & frame_id,
  const std::string & path_topic,
  const std::string & markers_topic,
  const std::string & interactive_marker_namespace,
  std::function<bool()> planning_enabled_cb)
{
  node_ = node;
  frame_id_ = frame_id;
  planning_enabled_cb_ = std::move(planning_enabled_cb);

  auto parameters_interface = node_->get_node_parameters_interface();
  auto topics_interface = node_->get_node_topics_interface();
  path_pub_ = rclcpp::create_publisher<PathMsg>(
    parameters_interface,
    topics_interface,
    path_topic,
    rclcpp::SystemDefaultsQoS());
  markers_pub_ = rclcpp::create_publisher<MarkerArrayMsg>(
    parameters_interface,
    topics_interface,
    markers_topic,
    rclcpp::SystemDefaultsQoS());

  interactive_marker_server_ = std::make_unique<InteractiveMarkerServer>(
    interactive_marker_namespace,
    node_->get_node_base_interface(),
    node_->get_node_clock_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_topics_interface(),
    node_->get_node_services_interface());

  publishPreview();
}

void PathEditor::cleanup()
{
  if (interactive_marker_server_) {
    interactive_marker_server_->clear();
    interactive_marker_server_->applyChanges();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoints_.clear();
  }

  interactive_marker_server_.reset();
  path_pub_.reset();
  markers_pub_.reset();
  planning_enabled_cb_ = nullptr;
  node_ = nullptr;
}

bool PathEditor::addWaypoint(
  const PathWaypoint & waypoint,
  int32_t & index,
  std::string & message)
{
  if (!planning_enabled_cb_ || !planning_enabled_cb_()) {
    message = "Path editing is only allowed while the lifecycle node is inactive";
    return false;
  }
  if (!isFiniteWaypoint(waypoint)) {
    message = "Waypoint values must be finite";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoints_.push_back(waypoint);
    index = static_cast<int32_t>(waypoints_.size() - 1U);
  }

  rebuildInteractiveMarkers();
  publishPreview();
  message = "Waypoint added";
  return true;
}

bool PathEditor::removeLastWaypoint(std::string & message)
{
  if (!planning_enabled_cb_ || !planning_enabled_cb_()) {
    message = "Path editing is only allowed while the lifecycle node is inactive";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (waypoints_.empty()) {
      message = "Path is already empty";
      return false;
    }
    waypoints_.pop_back();
  }

  rebuildInteractiveMarkers();
  publishPreview();
  message = "Last waypoint removed";
  return true;
}

bool PathEditor::clearPath(std::string & message)
{
  if (!planning_enabled_cb_ || !planning_enabled_cb_()) {
    message = "Path editing is only allowed while the lifecycle node is inactive";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoints_.clear();
  }

  rebuildInteractiveMarkers();
  publishPreview();
  message = "Path cleared";
  return true;
}

bool PathEditor::savePath(const std::string & path_file, std::string & message) const
{
  if (!planning_enabled_cb_ || !planning_enabled_cb_()) {
    message = "Saving paths is only allowed while the lifecycle node is inactive";
    return false;
  }
  if (path_file.empty()) {
    message = "path_file cannot be empty";
    return false;
  }

  std::vector<PathWaypoint> waypoints;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoints = waypoints_;
  }
  if (waypoints.empty()) {
    message = "Cannot save an empty path";
    return false;
  }

  tinyxml2::XMLDocument document;
  auto * root = document.NewElement("path");
  root->SetAttribute("frame_id", frame_id_.c_str());
  document.InsertFirstChild(root);

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    auto * waypoint_xml = document.NewElement("waypoint");
    waypoint_xml->SetAttribute("index", static_cast<int>(i));
    waypoint_xml->SetAttribute("x", waypoints[i].x);
    waypoint_xml->SetAttribute("y", waypoints[i].y);
    waypoint_xml->SetAttribute("z", waypoints[i].z);
    waypoint_xml->SetAttribute("yaw", waypoints[i].yaw);
    root->InsertEndChild(waypoint_xml);
  }

  const auto result = document.SaveFile(path_file.c_str());
  if (result != tinyxml2::XML_SUCCESS) {
    message = "Failed to save XML path: " + std::string(document.ErrorStr());
    return false;
  }

  message = "Path saved";
  return true;
}

bool PathEditor::loadPath(const std::string & path_file, std::string & message)
{
  if (!planning_enabled_cb_ || !planning_enabled_cb_()) {
    message = "Loading paths is only allowed while the lifecycle node is inactive";
    return false;
  }

  std::vector<PathWaypoint> loaded_waypoints;
  std::string loaded_frame_id;
  if (!loadPathForExecution(path_file, loaded_waypoints, loaded_frame_id, message)) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_id_ = loaded_frame_id.empty() ? frame_id_ : loaded_frame_id;
    waypoints_ = std::move(loaded_waypoints);
  }

  rebuildInteractiveMarkers();
  publishPreview();
  message = "Path loaded";
  return true;
}

bool PathEditor::loadPathForExecution(
  const std::string & path_file,
  std::vector<PathWaypoint> & waypoints,
  std::string & frame_id,
  std::string & message) const
{
  if (path_file.empty()) {
    message = "path_file cannot be empty";
    return false;
  }

  tinyxml2::XMLDocument document;
  const auto load_result = document.LoadFile(path_file.c_str());
  if (load_result != tinyxml2::XML_SUCCESS) {
    message = "Failed to load XML path: " + std::string(document.ErrorStr());
    return false;
  }

  const auto * root = document.FirstChildElement("path");
  if (!root) {
    message = "XML path is missing the root <path> element";
    return false;
  }

  const char * frame_id_attribute = root->Attribute("frame_id");
  frame_id = frame_id_attribute && std::string(frame_id_attribute).size() > 0 ?
    std::string(frame_id_attribute) : frame_id_;

  std::vector<PathWaypoint> parsed_waypoints;
  int expected_index = 0;
  for (const auto * waypoint_xml = root->FirstChildElement("waypoint"); waypoint_xml;
    waypoint_xml = waypoint_xml->NextSiblingElement("waypoint"))
  {
    int index = -1;
    PathWaypoint waypoint;
    if (
      waypoint_xml->QueryIntAttribute("index", &index) != tinyxml2::XML_SUCCESS ||
      waypoint_xml->QueryDoubleAttribute("x", &waypoint.x) != tinyxml2::XML_SUCCESS ||
      waypoint_xml->QueryDoubleAttribute("y", &waypoint.y) != tinyxml2::XML_SUCCESS ||
      waypoint_xml->QueryDoubleAttribute("z", &waypoint.z) != tinyxml2::XML_SUCCESS ||
      waypoint_xml->QueryDoubleAttribute("yaw", &waypoint.yaw) != tinyxml2::XML_SUCCESS)
    {
      message = "Each waypoint must define index, x, y, z, and yaw attributes";
      return false;
    }
    if (index != expected_index) {
      message = "Waypoint indices must start at 0 and be contiguous";
      return false;
    }
    if (!isFiniteWaypoint(waypoint)) {
      message = "Waypoint values must be finite";
      return false;
    }

    parsed_waypoints.push_back(waypoint);
    ++expected_index;
  }

  if (parsed_waypoints.empty()) {
    message = "XML path contains no waypoints";
    return false;
  }

  waypoints = std::move(parsed_waypoints);
  return true;
}

void PathEditor::publishPreview() const
{
  publishPath();
  publishMarkers();
}

void PathEditor::rebuildInteractiveMarkers()
{
  std::lock_guard<std::mutex> lock(mutex_);
  rebuildInteractiveMarkersLocked();
}

void PathEditor::handleMarkerFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (!planning_enabled_cb_ || !planning_enabled_cb_()) {
    return;
  }
  if (!feedback || feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    return;
  }

  const std::string prefix = "waypoint_";
  if (feedback->marker_name.rfind(prefix, 0) != 0) {
    return;
  }

  const auto index_text = feedback->marker_name.substr(prefix.size());
  std::size_t parsed_chars = 0;
  std::size_t index = 0;
  try {
    index = static_cast<std::size_t>(std::stoul(index_text, &parsed_chars));
  } catch (const std::exception &) {
    return;
  }
  if (parsed_chars != index_text.size()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index >= waypoints_.size() || !isFinitePose(feedback->pose)) {
      return;
    }

    waypoints_[index].x = feedback->pose.position.x;
    waypoints_[index].y = feedback->pose.position.y;
    waypoints_[index].z = feedback->pose.position.z;
    waypoints_[index].yaw = quaternionToYaw(feedback->pose.orientation);
  }

  publishPreview();
}

void PathEditor::publishPath() const
{
  if (!path_pub_) {
    return;
  }
  path_pub_->publish(toPathMessageLocked());
}

void PathEditor::publishMarkers() const
{
  if (!markers_pub_) {
    return;
  }
  markers_pub_->publish(toMarkerArrayLocked());
}

PathEditor::PathMsg PathEditor::toPathMessageLocked() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  PathMsg path;
  path.header.frame_id = frame_id_;
  path.header.stamp = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
  path.poses.reserve(waypoints_.size());

  for (const auto & waypoint : waypoints_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = waypoint.x;
    pose.pose.position.y = waypoint.y;
    pose.pose.position.z = waypoint.z;
    pose.pose.orientation = yawToQuaternion(waypoint.yaw);
    path.poses.push_back(pose);
  }

  return path;
}

PathEditor::MarkerArrayMsg PathEditor::toMarkerArrayLocked() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  MarkerArrayMsg marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  const auto stamp = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);

  visualization_msgs::msg::Marker line_marker;
  line_marker.header.frame_id = frame_id_;
  line_marker.header.stamp = stamp;
  line_marker.ns = "path_line";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = 0.04;
  line_marker.color.r = 0.0F;
  line_marker.color.g = 0.7F;
  line_marker.color.b = 1.0F;
  line_marker.color.a = 1.0F;

  for (const auto & waypoint : waypoints_) {
    geometry_msgs::msg::Point point;
    point.x = waypoint.x;
    point.y = waypoint.y;
    point.z = waypoint.z;
    line_marker.points.push_back(point);
  }
  marker_array.markers.push_back(line_marker);

  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    const auto & waypoint = waypoints_[i];

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id_;
    text_marker.header.stamp = stamp;
    text_marker.ns = "waypoint_text";
    text_marker.id = static_cast<int>(i);
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = waypoint.x;
    text_marker.pose.position.y = waypoint.y;
    text_marker.pose.position.z = waypoint.z + 0.25;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.25;
    text_marker.color.r = 1.0F;
    text_marker.color.g = 1.0F;
    text_marker.color.b = 1.0F;
    text_marker.color.a = 1.0F;
    text_marker.text = std::to_string(i);
    marker_array.markers.push_back(text_marker);

    visualization_msgs::msg::Marker yaw_marker;
    yaw_marker.header.frame_id = frame_id_;
    yaw_marker.header.stamp = stamp;
    yaw_marker.ns = "waypoint_yaw";
    yaw_marker.id = static_cast<int>(i);
    yaw_marker.type = visualization_msgs::msg::Marker::ARROW;
    yaw_marker.action = visualization_msgs::msg::Marker::ADD;
    yaw_marker.pose.position.x = waypoint.x;
    yaw_marker.pose.position.y = waypoint.y;
    yaw_marker.pose.position.z = waypoint.z;
    yaw_marker.pose.orientation = yawToQuaternion(waypoint.yaw);
    yaw_marker.scale.x = 0.45;
    yaw_marker.scale.y = 0.08;
    yaw_marker.scale.z = 0.08;
    yaw_marker.color.r = 1.0F;
    yaw_marker.color.g = 0.75F;
    yaw_marker.color.b = 0.0F;
    yaw_marker.color.a = 1.0F;
    marker_array.markers.push_back(yaw_marker);
  }

  return marker_array;
}

void PathEditor::rebuildInteractiveMarkersLocked()
{
  if (!interactive_marker_server_) {
    return;
  }

  interactive_marker_server_->clear();
  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    const auto & waypoint = waypoints_[i];

    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    marker.name = markerName(i);
    marker.description = "Waypoint " + std::to_string(i);
    marker.scale = 0.7;
    marker.pose.position.x = waypoint.x;
    marker.pose.position.y = waypoint.y;
    marker.pose.position.z = waypoint.z;
    marker.pose.orientation = yawToQuaternion(waypoint.yaw);

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

    visualization_msgs::msg::InteractiveMarkerControl move_plane_control;
    move_plane_control.name = "move_xy";
    move_plane_control.orientation.w = 1.0;
    move_plane_control.orientation.x = 0.0;
    move_plane_control.orientation.y = 1.0;
    move_plane_control.orientation.z = 0.0;
    move_plane_control.orientation_mode =
      visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    move_plane_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    marker.controls.push_back(move_plane_control);

    visualization_msgs::msg::InteractiveMarkerControl move_x_control;
    move_x_control.name = "move_x";
    move_x_control.orientation.w = 1.0;
    move_x_control.orientation.x = 1.0;
    move_x_control.orientation.y = 0.0;
    move_x_control.orientation.z = 0.0;
    move_x_control.orientation_mode =
      visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    move_x_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(move_x_control);

    visualization_msgs::msg::InteractiveMarkerControl move_y_control;
    move_y_control.name = "move_y";
    move_y_control.orientation.w = 1.0;
    move_y_control.orientation.x = 0.0;
    move_y_control.orientation.y = 0.0;
    move_y_control.orientation.z = 1.0;
    move_y_control.orientation_mode =
      visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    move_y_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(move_y_control);

    visualization_msgs::msg::InteractiveMarkerControl move_z_control;
    move_z_control.name = "move_z";
    move_z_control.orientation.w = 1.0;
    move_z_control.orientation.x = 0.0;
    move_z_control.orientation.y = 1.0;
    move_z_control.orientation.z = 0.0;
    move_z_control.orientation_mode =
      visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    move_z_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(move_z_control);

    visualization_msgs::msg::InteractiveMarkerControl rotate_yaw_control;
    rotate_yaw_control.name = "rotate_yaw";
    rotate_yaw_control.orientation.w = 1.0;
    rotate_yaw_control.orientation.x = 0.0;
    rotate_yaw_control.orientation.y = 1.0;
    rotate_yaw_control.orientation.z = 0.0;
    rotate_yaw_control.orientation_mode =
      visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    rotate_yaw_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(rotate_yaw_control);

    interactive_marker_server_->insert(
      marker,
      std::bind(&PathEditor::handleMarkerFeedback, this, std::placeholders::_1));
  }
  interactive_marker_server_->applyChanges();
}

void FollowPathExecutor::configure(
  rclcpp_lifecycle::LifecycleNode * node,
  const std::string & navigator_topic,
  const std::string & body_velocity_command_topic,
  const std::string & depth_setpoint_topic,
  const std::string & set_control_mode_service,
  double default_max_vertical_speed,
  bool default_holonomic,
  double default_goal_tolerance,
  double default_yaw_tolerance,
  double default_max_forward_speed,
  double default_max_yaw_rate,
  double default_timeout,
  double navigator_timeout,
  double mode_request_timeout,
  double control_loop_rate,
  double gain_x,
  double gain_y,
  double gain_z,
  double gain_yaw)
{
  node_ = node;
  default_max_vertical_speed_ = sanitizePositive(default_max_vertical_speed, 0.3);
  default_holonomic_ = default_holonomic;
  default_goal_tolerance_ = sanitizePositive(default_goal_tolerance, 0.2);
  default_yaw_tolerance_ = sanitizePositive(default_yaw_tolerance, 0.1);
  default_max_forward_speed_ = sanitizePositive(default_max_forward_speed, 0.2);
  default_max_yaw_rate_ = sanitizePositive(default_max_yaw_rate, 0.3);
  default_timeout_ = sanitizePositive(default_timeout, 60.0);
  navigator_timeout_ = sanitizePositive(navigator_timeout, 2.0);
  mode_request_timeout_ = sanitizePositive(mode_request_timeout, 5.0);
  control_loop_rate_ = sanitizePositive(control_loop_rate, 15.0);
  gain_x_ = gain_x;
  gain_y_ = gain_y;
  gain_z_ = gain_z;
  gain_yaw_ = gain_yaw;
  stop_requested_ = false;
  executing_ = false;

  navigator_sub_ = node_->create_subscription<NavigatorMsg>(
    navigator_topic,
    rclcpp::SystemDefaultsQoS(),
    [this](NavigatorMsg::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(navigator_mutex_);
      last_navigator_msg_ = std::move(msg);
      last_navigator_time_ = node_->now();
    });

  auto parameters_interface = node_->get_node_parameters_interface();
  auto topics_interface = node_->get_node_topics_interface();
  body_velocity_pub_ = rclcpp::create_publisher<TwistMsg>(
    parameters_interface,
    topics_interface,
    body_velocity_command_topic,
    rclcpp::SystemDefaultsQoS());
  depth_setpoint_pub_ = rclcpp::create_publisher<DepthSetPointMsg>(
    parameters_interface,
    topics_interface,
    depth_setpoint_topic,
    rclcpp::SystemDefaultsQoS());

  set_control_mode_client_ = node_->create_client<SetControlMode>(set_control_mode_service);
}

void FollowPathExecutor::cleanup()
{
  requestStop();
  joinExecutionThread();
  publishZeroVelocity();
  navigator_sub_.reset();
  body_velocity_pub_.reset();
  depth_setpoint_pub_.reset();
  set_control_mode_client_.reset();
  {
    std::lock_guard<std::mutex> lock(navigator_mutex_);
    last_navigator_msg_.reset();
  }
  node_ = nullptr;
}

bool FollowPathExecutor::isExecuting() const
{
  return executing_.load();
}

bool FollowPathExecutor::tryStartExecution()
{
  bool expected = false;
  if (!executing_.compare_exchange_strong(expected, true)) {
    return false;
  }
  stop_requested_ = false;
  return true;
}

void FollowPathExecutor::finishExecution()
{
  executing_ = false;
}

void FollowPathExecutor::requestStop()
{
  stop_requested_ = true;
  publishZeroVelocity();
}

void FollowPathExecutor::joinExecutionThread()
{
  std::thread thread_to_join;
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    if (execution_thread_.joinable() && execution_thread_.get_id() != std::this_thread::get_id()) {
      thread_to_join = std::move(execution_thread_);
    }
  }
  if (thread_to_join.joinable()) {
    thread_to_join.join();
  }
}

void FollowPathExecutor::startExecutionThread(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle,
  std::vector<PathWaypoint> waypoints)
{
  joinExecutionThread();
  std::lock_guard<std::mutex> lock(execution_mutex_);
  execution_thread_ = std::thread(
    &FollowPathExecutor::execute,
    this,
    goal_handle,
    std::move(waypoints));
}

std::vector<PathWaypoint> FollowPathExecutor::pathFromGoal(const FollowPath::Goal & goal) const
{
  std::vector<PathWaypoint> waypoints;
  waypoints.reserve(goal.path.poses.size());
  for (const auto & pose_stamped : goal.path.poses) {
    PathWaypoint waypoint;
    waypoint.x = pose_stamped.pose.position.x;
    waypoint.y = pose_stamped.pose.position.y;
    waypoint.z = pose_stamped.pose.position.z;
    waypoint.yaw = quaternionToYaw(pose_stamped.pose.orientation);
    waypoints.push_back(waypoint);
  }
  return waypoints;
}

void FollowPathExecutor::execute(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle,
  std::vector<PathWaypoint> waypoints)
{
  const auto finish_guard = std::unique_ptr<void, std::function<void(void *)>>(
    reinterpret_cast<void *>(1),
    [this](void *) { finishExecution(); });
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<FollowPath::Result>();
  auto feedback = std::make_shared<FollowPath::Feedback>();

  const auto fail = [this, &goal_handle, &result](const std::string & message)
    {
      publishZeroVelocity();
      result->success = false;
      result->message = message;
      goal_handle->abort(result);
    };

  if (!node_) {
    result->success = false;
    result->message = "FollowPath executor is not configured";
    goal_handle->abort(result);
    return;
  }
  if (waypoints.empty()) {
    fail("FollowPath goal contains no waypoints");
    return;
  }
  for (const auto & waypoint : waypoints) {
    if (!isFiniteWaypoint(waypoint)) {
      fail("FollowPath goal contains a non-finite waypoint");
      return;
    }
  }

  const double goal_tolerance = sanitizePositive(goal->goal_tolerance, default_goal_tolerance_);
  const double yaw_tolerance = sanitizePositive(goal->yaw_tolerance, default_yaw_tolerance_);
  const double max_forward_speed =
    sanitizePositive(goal->max_forward_speed, default_max_forward_speed_);
  const double max_yaw_rate = sanitizePositive(goal->max_yaw_rate, default_max_yaw_rate_);
  const double timeout = sanitizePositive(goal->timeout, default_timeout_);
  const bool holonomic = goal->holonomic || default_holonomic_;
  const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(timeout);
  const rclcpp::Duration navigator_timeout_duration =
    rclcpp::Duration::from_seconds(navigator_timeout_);
  const auto start_time = node_->now();

  std::string mode_error;
  if (!requestControlMode(
      SetControlMode::Request::FOLLOW_PATH,
      "FollowPath action goal accepted",
      mode_error))
  {
    fail("Failed to request FOLLOW_PATH mode: " + mode_error);
    return;
  }

  rclcpp::Rate loop_rate(control_loop_rate_);
  std::size_t current_waypoint_index = 0;
  while (rclcpp::ok() && current_waypoint_index < waypoints.size()) {
    if (goal_handle->is_canceling()) {
      publishZeroVelocity();
      result->success = false;
      result->message = "FollowPath goal canceled";
      goal_handle->canceled(result);
      return;
    }
    if (stop_requested_.load()) {
      publishZeroVelocity();
      result->success = false;
      result->message = "FollowPath execution stopped";
      goal_handle->abort(result);
      return;
    }

    NavigatorMsg navigator_msg;
    rclcpp::Time navigator_stamp;
    const bool has_navigation = getNavigatorSnapshot(navigator_msg, navigator_stamp);
    const auto now = node_->now();
    if (!has_navigation || (now - navigator_stamp) > navigator_timeout_duration) {
      fail(has_navigation ? "Navigator data timed out" : "Navigator data was not received");
      return;
    }

    if ((now - start_time) > timeout_duration) {
      fail("FollowPath goal timed out");
      return;
    }

    const auto & target = waypoints[current_waypoint_index];
    const auto & current_position = navigator_msg.position.position;
    const double error_x = target.x - current_position.x;
    const double error_y = target.y - current_position.y;
    const double error_z = target.z - current_position.z;
    const double distance_to_target = std::hypot(std::hypot(error_x, error_y), error_z);
    const double current_yaw = quaternionToYaw(navigator_msg.position.orientation);
    const double desired_yaw = holonomic ? target.yaw : std::atan2(error_y, error_x);
    const double yaw_error = normalizeAngle(desired_yaw - current_yaw);

    if (distance_to_target <= goal_tolerance && std::abs(yaw_error) <= yaw_tolerance) {
      ++current_waypoint_index;
      publishZeroVelocity();
      continue;
    }

    const double yaw_rate = std::clamp(gain_yaw_ * yaw_error, -max_yaw_rate, max_yaw_rate);
    const double vertical_speed = std::clamp(
      gain_z_ * error_z,
      -default_max_vertical_speed_,
      default_max_vertical_speed_);
    double forward_speed = 0.0;
    double lateral_speed = 0.0;

    if (holonomic) {
      const double cos_yaw = std::cos(current_yaw);
      const double sin_yaw = std::sin(current_yaw);
      const double body_error_x = cos_yaw * error_x + sin_yaw * error_y;
      const double body_error_y = -sin_yaw * error_x + cos_yaw * error_y;
      forward_speed = std::clamp(gain_x_ * body_error_x, -max_forward_speed, max_forward_speed);
      lateral_speed = std::clamp(gain_y_ * body_error_y, -max_forward_speed, max_forward_speed);
    } else {
      const double yaw_alignment_scale =
        std::clamp(1.0 - std::abs(yaw_error) / std::max(yaw_tolerance, 0.001), 0.0, 1.0);
      forward_speed = max_forward_speed * yaw_alignment_scale;
    }

    publishBodyVelocity(forward_speed, lateral_speed, vertical_speed, yaw_rate);
    publishDepthSetpoint(target.z);

    feedback->current_x = current_position.x;
    feedback->current_y = current_position.y;
    feedback->current_z = current_position.z;
    feedback->current_yaw = current_yaw;
    feedback->current_waypoint_index = static_cast<int32_t>(current_waypoint_index);
    feedback->distance_to_target = distance_to_target;
    feedback->progress = static_cast<double>(current_waypoint_index) /
      static_cast<double>(waypoints.size());
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  publishZeroVelocity();
  result->success = true;
  result->message = "FollowPath target reached";
  goal_handle->succeed(result);
}

bool FollowPathExecutor::getNavigatorSnapshot(
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

bool FollowPathExecutor::requestControlMode(
  uint8_t mode,
  const std::string & reason,
  std::string & error_message)
{
  if (!set_control_mode_client_) {
    error_message = "set_mode client is not configured";
    return false;
  }
  if (!set_control_mode_client_->wait_for_service(std::chrono::duration<double>(mode_request_timeout_))) {
    error_message = "set_mode service is not available";
    return false;
  }

  auto request = std::make_shared<SetControlMode::Request>();
  request->mode = mode;
  request->reason = reason;

  auto future = set_control_mode_client_->async_send_request(request);
  const auto status = future.wait_for(std::chrono::duration<double>(mode_request_timeout_ + 1.0));
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

void FollowPathExecutor::publishBodyVelocity(
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

void FollowPathExecutor::publishDepthSetpoint(double target_depth)
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

void FollowPathExecutor::publishZeroVelocity()
{
  publishBodyVelocity(0.0, 0.0, 0.0, 0.0);
}

PathManagerLifecycleNode::PathManagerLifecycleNode(const rclcpp::NodeOptions & options)
: LifecycleNode("path_manager_lifecycle_node", options)
{
}

PathManagerLifecycleNode::~PathManagerLifecycleNode()
{
  cleanupResources();
}

PathManagerLifecycleNode::CallbackReturn PathManagerLifecycleNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  robot_namespace_ = getOrDeclareParameter<std::string>(*this, "robot_namespace", "sura");
  frame_id_ = getOrDeclareParameter<std::string>(*this, "frame_id", "world_ned");
  path_topic_ = getOrDeclareParameter<std::string>(
    *this,
    "path_topic", namespacedTopic("path_manager/path"));
  markers_topic_ = getOrDeclareParameter<std::string>(
    *this,
    "markers_topic", namespacedTopic("path_manager/markers"));
  interactive_marker_namespace_ = getOrDeclareParameter<std::string>(
    *this,
    "interactive_marker_namespace", namespacedTopic("path_manager/interactive_markers"));
  action_name_ = getOrDeclareParameter<std::string>(
    *this,
    "action_name", namespacedTopic("actions/follow_path"));
  navigator_topic_ = getOrDeclareParameter<std::string>(
    *this,
    "navigator_topic", namespacedTopic("navigator/navigation"));
  body_velocity_command_topic_ = getOrDeclareParameter<std::string>(
    *this,
    "body_velocity_command_topic", namespacedTopic("controller/body_velocity/setpoint"));
  depth_setpoint_topic_ = getOrDeclareParameter<std::string>(
    *this,
    "depth_setpoint_topic", namespacedTopic("controller/depth_hold/set_point"));
  set_control_mode_service_ = getOrDeclareParameter<std::string>(
    *this,
    "set_control_mode_service", namespacedTopic("control_manager/set_mode"));

  const double default_max_vertical_speed =
    getOrDeclareParameter<double>(*this, "default_max_vertical_speed", 0.3);
  default_holonomic_ = getOrDeclareParameter<bool>(*this, "defaults.holonomic", false);
  default_goal_tolerance_ = getOrDeclareParameter<double>(*this, "defaults.goal_tolerance", 0.2);
  default_yaw_tolerance_ = getOrDeclareParameter<double>(*this, "defaults.yaw_tolerance", 0.1);
  default_max_forward_speed_ =
    getOrDeclareParameter<double>(*this, "defaults.max_forward_speed", 0.2);
  default_max_yaw_rate_ = getOrDeclareParameter<double>(*this, "defaults.max_yaw_rate", 0.3);
  default_timeout_ = getOrDeclareParameter<double>(*this, "defaults.timeout", 60.0);
  const double navigator_timeout = getOrDeclareParameter<double>(*this, "navigator_timeout", 2.0);
  const double mode_request_timeout =
    getOrDeclareParameter<double>(*this, "mode_request_timeout", 5.0);
  const double control_loop_rate = getOrDeclareParameter<double>(*this, "control_loop_rate", 15.0);
  const double gain_x = getOrDeclareParameter<double>(*this, "gains.x", 0.4);
  const double gain_y = getOrDeclareParameter<double>(*this, "gains.y", 0.4);
  const double gain_z = getOrDeclareParameter<double>(*this, "gains.z", 1.0);
  const double gain_yaw = getOrDeclareParameter<double>(*this, "gains.yaw", 1.0);

  editor_.configure(
    this,
    frame_id_,
    path_topic_,
    markers_topic_,
    interactive_marker_namespace_,
    [this]() { return isInactive(); });
  executor_.configure(
    this,
    navigator_topic_,
    body_velocity_command_topic_,
    depth_setpoint_topic_,
    set_control_mode_service_,
    default_max_vertical_speed,
    default_holonomic_,
    default_goal_tolerance_,
    default_yaw_tolerance_,
    default_max_forward_speed_,
    default_max_yaw_rate_,
    default_timeout_,
    navigator_timeout,
    mode_request_timeout,
    control_loop_rate,
    gain_x,
    gain_y,
    gain_z,
    gain_yaw);

  add_waypoint_srv_ = this->create_service<srv::AddWaypoint>(
    namespacedTopic("path_manager/add_waypoint"),
    std::bind(
      &PathManagerLifecycleNode::handleAddWaypoint,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  remove_last_waypoint_srv_ = this->create_service<std_srvs::srv::Trigger>(
    namespacedTopic("path_manager/remove_last_waypoint"),
    std::bind(
      &PathManagerLifecycleNode::handleRemoveLastWaypoint,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  clear_path_srv_ = this->create_service<std_srvs::srv::Trigger>(
    namespacedTopic("path_manager/clear_path"),
    std::bind(
      &PathManagerLifecycleNode::handleClearPath,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  save_path_srv_ = this->create_service<srv::PathFile>(
    namespacedTopic("path_manager/save_path"),
    std::bind(
      &PathManagerLifecycleNode::handleSavePath,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  load_path_srv_ = this->create_service<srv::PathFile>(
    namespacedTopic("path_manager/load_path"),
    std::bind(
      &PathManagerLifecycleNode::handleLoadPath,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  action_server_ = rclcpp_action::create_server<FollowPath>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    action_name_,
    std::bind(
      &PathManagerLifecycleNode::handleGoal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&PathManagerLifecycleNode::handleCancel, this, std::placeholders::_1),
    std::bind(&PathManagerLifecycleNode::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "PathManagerLifecycleNode configured");
  return CallbackReturn::SUCCESS;
}

PathManagerLifecycleNode::CallbackReturn PathManagerLifecycleNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  editor_.publishPreview();
  RCLCPP_INFO(get_logger(), "PathManagerLifecycleNode active: FollowPath goals are accepted");
  return CallbackReturn::SUCCESS;
}

PathManagerLifecycleNode::CallbackReturn PathManagerLifecycleNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  executor_.requestStop();
  executor_.joinExecutionThread();
  editor_.publishPreview();
  RCLCPP_INFO(get_logger(), "PathManagerLifecycleNode inactive: path planning is enabled");
  return CallbackReturn::SUCCESS;
}

PathManagerLifecycleNode::CallbackReturn PathManagerLifecycleNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

PathManagerLifecycleNode::CallbackReturn PathManagerLifecycleNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

bool PathManagerLifecycleNode::isInactive()
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

bool PathManagerLifecycleNode::isActive()
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

std::string PathManagerLifecycleNode::namespacedTopic(const std::string & suffix) const
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

void PathManagerLifecycleNode::cleanupResources()
{
  action_server_.reset();
  add_waypoint_srv_.reset();
  remove_last_waypoint_srv_.reset();
  clear_path_srv_.reset();
  save_path_srv_.reset();
  load_path_srv_.reset();
  executor_.cleanup();
  editor_.cleanup();
}

PathManagerLifecycleNode::FollowPath::Goal PathManagerLifecycleNode::resolveGoalDefaults(
  const FollowPath::Goal & goal) const
{
  auto resolved_goal = goal;
  resolved_goal.holonomic = goal.holonomic || default_holonomic_;
  resolved_goal.goal_tolerance = sanitizePositive(goal.goal_tolerance, default_goal_tolerance_);
  resolved_goal.yaw_tolerance = sanitizePositive(goal.yaw_tolerance, default_yaw_tolerance_);
  resolved_goal.max_forward_speed =
    sanitizePositive(goal.max_forward_speed, default_max_forward_speed_);
  resolved_goal.max_yaw_rate = sanitizePositive(goal.max_yaw_rate, default_max_yaw_rate_);
  resolved_goal.timeout = sanitizePositive(goal.timeout, default_timeout_);
  return resolved_goal;
}

rclcpp_action::GoalResponse PathManagerLifecycleNode::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const FollowPath::Goal> goal)
{
  if (!isActive()) {
    RCLCPP_WARN(get_logger(), "Rejecting FollowPath goal because the node is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (executor_.isExecuting()) {
    RCLCPP_WARN(get_logger(), "Rejecting FollowPath goal because another goal is executing");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto resolved_goal = resolveGoalDefaults(*goal);
  if (
    resolved_goal.goal_tolerance <= 0.0 ||
    resolved_goal.yaw_tolerance <= 0.0 ||
    resolved_goal.max_forward_speed <= 0.0 ||
    resolved_goal.max_yaw_rate <= 0.0 ||
    resolved_goal.timeout <= 0.0)
  {
    RCLCPP_WARN(get_logger(), "Rejecting FollowPath goal because limits and tolerances must be positive");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->use_saved_path && goal->path_file.empty()) {
    RCLCPP_WARN(get_logger(), "Rejecting FollowPath goal because path_file is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal->use_saved_path && goal->path.poses.empty()) {
    RCLCPP_WARN(get_logger(), "Rejecting FollowPath goal because path is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathManagerLifecycleNode::handleCancel(
  const std::shared_ptr<GoalHandleFollowPath>)
{
  executor_.requestStop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PathManagerLifecycleNode::handleAccepted(
  const std::shared_ptr<GoalHandleFollowPath> goal_handle)
{
  if (!executor_.tryStartExecution()) {
    auto result = std::make_shared<FollowPath::Result>();
    result->success = false;
    result->message = "Another FollowPath goal is already executing";
    goal_handle->abort(result);
    return;
  }

  const auto goal = goal_handle->get_goal();
  std::vector<PathWaypoint> waypoints;
  if (goal->use_saved_path) {
    std::string frame_id;
    std::string message;
    if (!editor_.loadPathForExecution(goal->path_file, waypoints, frame_id, message)) {
      auto result = std::make_shared<FollowPath::Result>();
      result->success = false;
      result->message = message;
      goal_handle->abort(result);
      executor_.finishExecution();
      return;
    }
  } else {
    waypoints = executor_.pathFromGoal(*goal);
  }

  executor_.startExecutionThread(goal_handle, std::move(waypoints));
}

void PathManagerLifecycleNode::handleAddWaypoint(
  const std::shared_ptr<srv::AddWaypoint::Request> request,
  std::shared_ptr<srv::AddWaypoint::Response> response)
{
  PathWaypoint waypoint{request->x, request->y, request->z, request->yaw};
  response->success = editor_.addWaypoint(waypoint, response->index, response->message);
}

void PathManagerLifecycleNode::handleRemoveLastWaypoint(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = editor_.removeLastWaypoint(response->message);
}

void PathManagerLifecycleNode::handleClearPath(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = editor_.clearPath(response->message);
}

void PathManagerLifecycleNode::handleSavePath(
  const std::shared_ptr<srv::PathFile::Request> request,
  std::shared_ptr<srv::PathFile::Response> response)
{
  response->success = editor_.savePath(request->path_file, response->message);
}

void PathManagerLifecycleNode::handleLoadPath(
  const std::shared_ptr<srv::PathFile::Request> request,
  std::shared_ptr<srv::PathFile::Response> response)
{
  response->success = editor_.loadPath(request->path_file, response->message);
}

}  // namespace sura_actions

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sura_actions::PathManagerLifecycleNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
