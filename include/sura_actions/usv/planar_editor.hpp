#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sura_actions/usv/path_io.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace sura_actions::usv
{
// All calls run in the owning node's mutually exclusive callback group.
class PlanarEditor
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode * node, const std::string & frame,
    const std::string & marker_namespace, const std::string & preview_topic,
    const std::string & markers_topic, bool single, std::function<bool()> editable)
  {
    node_ = node;
    frame_ = frame;
    single_ = single;
    editable_ = std::move(editable);
    auto parameters = node->get_node_parameters_interface();
    auto topics = node->get_node_topics_interface();
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      marker_namespace, node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_topics_interface(),
      node->get_node_services_interface());
    if (single) {
      pose_pub_ = rclcpp::create_publisher<geometry_msgs::msg::PoseStamped>(
        parameters, topics,
        preview_topic, rclcpp::QoS(1).transient_local());
      points_.resize(1);
    } else {
      path_pub_ = rclcpp::create_publisher<nav_msgs::msg::Path>(
        parameters, topics,
        preview_topic, rclcpp::QoS(1).transient_local());
      markers_pub_ = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
        parameters, topics,
        markers_topic, rclcpp::QoS(1).transient_local());
    }
    refresh();
  }

  const std::vector<Pose2D> & points() const {return points_;}
  bool hasPlannedPose() const {return planned_;}

  void setPoints(std::vector<Pose2D> points)
  {
    points_ = std::move(points);
    planned_ = !points_.empty();
    refresh();
  }

  void cleanup()
  {
    if (server_) {
      server_->clear();
      server_->applyChanges();
    }
    server_.reset();
    pose_pub_.reset();
    path_pub_.reset();
    markers_pub_.reset();
    points_.clear();
    planned_ = false;
    node_ = nullptr;
  }

private:
  void refresh()
  {
    if (!server_) {return;}
    server_->clear();
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_;
    path.header.stamp = node_->now();
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    visualization_msgs::msg::Marker line;
    line.header = path.header;
    line.ns = "path_line";
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.04;
    line.color.g = 0.7F;
    line.color.b = 1.0F;
    line.color.a = 1.0F;
    for (size_t i = 0; i < points_.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points_[i].x;
      pose.pose.position.y = points_[i].y;
      pose.pose.orientation = yawQuaternion(points_[i].theta);
      path.poses.push_back(pose);
      line.points.push_back(pose.pose.position);

      visualization_msgs::msg::InteractiveMarker marker;
      marker.header = path.header;
      marker.name = single_ ? "go_to_pose_goal" : "waypoint_" + std::to_string(i);
      marker.description = single_ ? "USV goal" : "Waypoint " + std::to_string(i);
      marker.pose = pose.pose;
      marker.scale = 0.7;
      visualization_msgs::msg::Marker arrow;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.pose.orientation.w = 1.0;
      arrow.scale.x = 0.45;
      arrow.scale.y = 0.08;
      arrow.scale.z = 0.08;
      arrow.color.r = 1.0F;
      arrow.color.g = 0.7F;
      arrow.color.a = 1.0F;
      visualization_msgs::msg::InteractiveMarkerControl visual;
      visual.always_visible = true;
      visual.markers.push_back(arrow);
      marker.controls.push_back(visual);
      using Control = visualization_msgs::msg::InteractiveMarkerControl;
      auto control = [&marker](
        const std::string & name, uint8_t mode, int axis, uint8_t orientation_mode)
        {
          Control c;
          c.name = name;
          c.orientation.w = std::sqrt(0.5);
          if (axis == 0) {c.orientation.x = std::sqrt(0.5);}
          if (axis == 1) {c.orientation.z = std::sqrt(0.5);}
          if (axis == 2) {c.orientation.y = std::sqrt(0.5);}
          c.orientation_mode = orientation_mode;
          c.interaction_mode = mode;
          marker.controls.push_back(c);
        };
      control("move_xy", Control::MOVE_PLANE, 2, Control::INHERIT);
      control("move_x", Control::MOVE_AXIS, 0, Control::INHERIT);
      control("move_y", Control::MOVE_AXIS, 1, Control::INHERIT);
      control("rotate_yaw", Control::ROTATE_AXIS, 2, Control::FIXED);
      server_->insert(
        marker, [this, i](auto feedback) {
          if (!editable_() || i >= points_.size() ||
          feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
          {return;}
          Pose2D point;
          point.x = feedback->pose.position.x;
          point.y = feedback->pose.position.y;
          if (!quaternionYaw(feedback->pose.orientation, point.theta) || !finitePose(point)) {
            return;
          }
          points_[i] = point;
          planned_ = true;
          refresh();
        });
      arrow.header = path.header;
      arrow.ns = "waypoint_yaw";
      arrow.id = static_cast<int>(i);
      arrow.pose = pose.pose;
      markers.markers.push_back(arrow);
      auto label = arrow;
      label.ns = "waypoint_text";
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position.z = 0.25;
      label.scale.z = 0.25;
      label.text = std::to_string(i);
      markers.markers.push_back(label);
    }
    server_->applyChanges();
    markers.markers.push_back(line);
    if (pose_pub_ && !path.poses.empty()) {pose_pub_->publish(path.poses.front());}
    if (path_pub_) {path_pub_->publish(path);}
    if (markers_pub_) {markers_pub_->publish(markers);}
  }

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  std::string frame_;
  bool single_{false};
  bool planned_{false};
  std::function<bool()> editable_;
  std::vector<Pose2D> points_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};
}  // namespace sura_actions::usv
