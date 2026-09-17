#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sura_actions/action/follow_path_usv.hpp"
#include "sura_actions/action/go_to_pose_usv.hpp"
#include "sura_actions/srv/add_waypoint_usv.hpp"
#include "sura_actions/srv/path_file.hpp"
#include "sura_actions/srv/set_control_mode.hpp"
#include "sura_actions/usv/planar_editor.hpp"
#include "sura_msgs/msg/navigator.hpp"
#include "sura_msgs/msg/sura_velocity_command.hpp"
#include "sura_msgs/srv/clear_controller_intents.hpp"

namespace sura_actions::usv
{
// Timer-based execution keeps lifecycle, cancellation, navigation and service
// responses serialized in the default mutually exclusive callback group.
template<class Action>
class PlanarActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using Mode = sura_actions::srv::SetControlMode;
  static constexpr bool single = std::is_same_v<Action, sura_actions::action::GoToPoseUsv>;

  explicit PlanarActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode(single ? "go_to_pose_usv_lifecycle_action_node" :
      "path_follower_usv_lifecycle_node", options) {}

  ~PlanarActionServer() override {cleanupResources();}

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    try {
      namespace_ = parameter<std::string>("robot_namespace", "sura");
      while (!namespace_.empty() && namespace_.front() == '/') {namespace_.erase(0, 1);}
      while (!namespace_.empty() && namespace_.back() == '/') {namespace_.pop_back();}
      frame_ = parameter<std::string>("frame_id", "world_ned");
      controller_ = parameter<std::string>("body_velocity_controller", "body_velocity");
      requester_ = parameter<std::string>("requester", single ? "go_to_pose" : "follow_path");
      priority_ = parameter<int>("priority", single ? 60 : 55);
      defaults_.forward_speed = positive("defaults.max_forward_speed", 0.20);
      defaults_.yaw_rate = positive("defaults.max_yaw_rate", 0.25);
      defaults_.position_tolerance = positive(
        single ? "defaults.position_tolerance" : "defaults.goal_tolerance", 0.30);
      defaults_.yaw_tolerance = positive("defaults.yaw_tolerance", 0.12);
      defaults_.slowdown_distance = positive("defaults.slowdown_distance", 1.2);
      defaults_.heading_stop = positive("misalignment_slowdown_yaw", 0.70);
      defaults_.yaw_gain = positive("gains.yaw", 0.8);
      default_timeout_ = positive("defaults.timeout", single ? 120.0 : 500.0);
      navigator_timeout_ = positive("navigator_timeout", 2.0);
      mode_timeout_ = positive("mode_request_timeout", 5.0);
      const double rate = positive("control_loop_rate", 15.0);
      if (frame_.empty() || controller_.empty() || requester_.empty() ||
        priority_ < 1 || priority_ > 100)
      {
        throw std::invalid_argument(
                "Frame/controller/requester must be non-empty; priority is 1..100");
      }
      const auto action_name = parameter<std::string>(
        "action_name",
        topic(single ? "actions/go_to_pose" : "actions/follow_path"));
      mode_client_ = create_client<Mode>(
        parameter<std::string>(
          "set_control_mode_service", topic("control_manager/set_mode")));
      clear_client_ = create_client<sura_msgs::srv::ClearControllerIntents>(
        parameter<std::string>(
          "clear_controller_intents_service",
          topic("controller/arbitrator/clear_controller_intents")));
      auto parameters = get_node_parameters_interface();
      auto topics = get_node_topics_interface();
      velocity_pub_ = rclcpp::create_publisher<sura_msgs::msg::SuraVelocityCommand>(
        parameters, topics,
        parameter<std::string>(
          "arbitrator_velocity_topic", topic(
            "controller/arbitrator/velocity")),
        rclcpp::SystemDefaultsQoS());
      navigator_sub_ = create_subscription<sura_msgs::msg::Navigator>(
        parameter<std::string>("navigator_topic", topic("navigator/navigation")),
        rclcpp::SystemDefaultsQoS(), [this](sura_msgs::msg::Navigator::ConstSharedPtr msg) {
          current_.x = msg->position.position.x;
          current_.y = msg->position.position.y;
          navigation_valid_ = quaternionYaw(msg->position.orientation, current_.theta) &&
          finitePose(current_);
          navigation_received_ = true;
          navigation_stamp_ = now();
        });
      editor_.configure(
        this, frame_, parameter<std::string>(
          "interactive_marker_namespace",
          topic(
            single ? "actions/go_to_pose/interactive_marker" :
            "path_manager/interactive_markers")),
        parameter<std::string>(
          single ? "target_pose_topic" : "path_topic",
          topic(single ? "actions/go_to_pose/target_pose" : "path_manager/path")),
        parameter<std::string>("markers_topic", topic("path_manager/markers")), single,
        [this]() {return inactive();});
      if constexpr (!single) {configurePathServices();}
      action_server_ = rclcpp_action::create_server<Action>(
        this, action_name,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename Action::Goal> goal) {
          if (!active() || goal_) {return rclcpp_action::GoalResponse::REJECT;}
          try {
            resolve(*goal);
          } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "Rejecting USV goal: %s", e.what());
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<Handle> handle) {
          return handle == goal_ ? rclcpp_action::CancelResponse::ACCEPT :
          rclcpp_action::CancelResponse::REJECT;
        },
        [this](std::shared_ptr<Handle> handle) {accept(handle);});
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / rate)), [this]() {tick();});
      RCLCPP_INFO(
        get_logger(), "Configured USV action %s in frame %s",
        action_name.c_str(), frame_.c_str());
      return CallbackReturn::SUCCESS;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "USV configuration failed: %s", e.what());
      cleanupResources();
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (goal_) {finish(false, "Lifecycle node deactivated", false);}
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    cleanupResources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    cleanupResources();
    return CallbackReturn::SUCCESS;
  }

private:
  template<class T> T parameter(const std::string & name, const T & fallback)
  {
    return has_parameter(name) ? get_parameter(name).get_value<T>() :
           declare_parameter<T>(name, fallback);
  }

  double positive(const std::string & name, double fallback)
  {
    const double value = parameter<double>(name, fallback);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  std::string topic(const std::string & suffix) const
  {
    return namespace_.empty() ? "/" + suffix : "/" + namespace_ + "/" + suffix;
  }

  bool active()
  {
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  bool inactive()
  {
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  }

  struct ResolvedGoal
  {
    std::vector<Pose2D> points;
    GuidanceLimits limits;
    double timeout;
  };

  ResolvedGoal resolve(const typename Action::Goal & goal)
  {
    ResolvedGoal resolved{{}, defaults_, goalValue(goal.timeout, default_timeout_)};
    resolved.limits.forward_speed = goalValue(goal.max_forward_speed, defaults_.forward_speed);
    resolved.limits.yaw_rate = goalValue(goal.max_yaw_rate, defaults_.yaw_rate);
    resolved.limits.yaw_tolerance = goalValue(goal.yaw_tolerance, defaults_.yaw_tolerance);
    if constexpr (single) {
      resolved.limits.position_tolerance = goalValue(
        goal.position_tolerance, defaults_.position_tolerance);
      resolved.limits.slowdown_distance = goalValue(
        goal.slowdown_distance, defaults_.slowdown_distance);
      if (goal.use_planned_pose) {
        if (!editor_.hasPlannedPose()) {throw std::invalid_argument("No planned marker pose");}
        resolved.points = editor_.points();
      } else {
        resolved.points = {goal.target_pose};
      }
    } else {
      resolved.limits.position_tolerance = goalValue(
        goal.goal_tolerance, defaults_.position_tolerance);
      if (goal.use_saved_path) {
        std::string error;
        if (!loadPath(goal.path_file, frame_, resolved.points, error)) {
          throw std::invalid_argument(error);
        }
      } else {
        resolved.points = goal.path;
      }
    }
    if (resolved.points.empty()) {throw std::invalid_argument("Path contains no waypoints");}
    for (auto & point : resolved.points) {
      if (!finitePose(point)) {throw std::invalid_argument("Goal pose must be finite");}
      point.theta = normalizeAngle(point.theta);
    }
    return resolved;
  }

  void accept(const std::shared_ptr<Handle> & handle)
  {
    if (goal_) {
      auto result = std::make_shared<typename Action::Result>();
      result->message = "Another goal is executing";
      handle->abort(result);
      return;
    }
    goal_ = handle;
    try {
      execution_ = resolve(*handle->get_goal());
    } catch (const std::exception & e) {
      finish(false, e.what());
      return;
    }
    waypoint_ = 0;
    started_ = now();
    mode_requested_ = false;
    mode_ready_ = false;
    editor_.setPoints(execution_.points);
  }

  void tick()
  {
    if (!goal_) {return;}
    if (goal_->is_canceling()) {finish(false, "USV goal canceled"); return;}
    const double elapsed = (now() - started_).seconds();
    if (elapsed >= execution_.timeout) {finish(false, "USV goal timed out"); return;}
    if (navigation_received_ && !navigation_valid_) {
      finish(false, "Navigator contains invalid planar pose"); return;
    }
    if (navigation_received_ && (now() - navigation_stamp_).seconds() > navigator_timeout_) {
      finish(false, "Navigator data timed out"); return;
    }
    if (!navigation_received_ && elapsed >= navigator_timeout_) {
      finish(false, "Navigator data was not received"); return;
    }
    if (!mode_ready_) {
      if (!mode_requested_ && mode_client_->service_is_ready()) {
        auto request = std::make_shared<Mode::Request>();
        request->mode = single ? Mode::Request::BODY_VELOCITY : Mode::Request::FOLLOW_PATH;
        request->reason = single ? "USV GoToPose" : "USV FollowPath";
        auto pending = mode_client_->async_send_request(request);
        mode_request_id_ = pending.request_id;
        mode_future_ = pending.share();
        mode_requested_ = true;
      }
      if (mode_requested_ && mode_future_.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready)
      {
        const auto response = mode_future_.get();
        mode_request_id_ = 0;
        if (!response->success) {
          finish(false, "Control mode failed: " + response->message); return;
        }
        mode_ready_ = true;
      } else {
        if (elapsed >= mode_timeout_) {finish(false, "Control mode request timed out"); return;}
        feedback(GuidanceOutput{}, "waiting_for_control_mode");
        return;
      }
    }
    if (!navigation_received_) {
      feedback(GuidanceOutput{}, "waiting_for_navigation");
      return;
    }
    auto output = guide(
      current_, execution_.points[waypoint_], execution_.limits,
      waypoint_ + 1 == execution_.points.size());
    // Passing intermediate waypoints does not insert a stop command.
    while (output.reached && waypoint_ + 1 < execution_.points.size()) {
      ++waypoint_;
      output = guide(
        current_, execution_.points[waypoint_], execution_.limits,
        waypoint_ + 1 == execution_.points.size());
    }
    feedback(
      output, output.reached ? "target_reached" :
      (output.aligning ? "aligning_final_yaw" : "moving"));
    if (output.reached) {finish(true, "USV target reached"); return;}
    publish(output.velocity);
  }

  void feedback(const GuidanceOutput & output, const std::string & state)
  {
    auto msg = std::make_shared<typename Action::Feedback>();
    msg->current_x = current_.x;
    msg->current_y = current_.y;
    msg->current_yaw = current_.theta;
    msg->state = state;
    if constexpr (single) {
      msg->distance_to_goal = output.distance;
      msg->yaw_error = output.yaw_error;
      msg->commanded_forward_speed = output.velocity.linear.x;
      msg->commanded_yaw_rate = output.velocity.angular.z;
    } else {
      msg->current_waypoint_index = static_cast<int32_t>(waypoint_);
      msg->distance_to_target = output.distance;
      msg->progress = output.reached ? 1.0 :
        static_cast<double>(waypoint_) / static_cast<double>(execution_.points.size());
    }
    goal_->publish_feedback(msg);
  }

  void publish(const geometry_msgs::msg::Twist & velocity)
  {
    if (!velocity_pub_) {return;}
    sura_msgs::msg::SuraVelocityCommand msg;
    msg.header.stamp = now();
    msg.requester = requester_;
    msg.controller = controller_;
    msg.priority = static_cast<uint8_t>(priority_);
    // Restrict commands at the final publication boundary as well.
    msg.velocity.linear.x = velocity.linear.x;
    msg.velocity.angular.z = velocity.angular.z;
    velocity_pub_->publish(msg);
  }

  void finish(bool success, const std::string & message, bool auto_deactivate = single)
  {
    publish(geometry_msgs::msg::Twist{});
    if (clear_client_ && clear_client_->service_is_ready()) {
      clear_client_->prune_pending_requests();
      auto request = std::make_shared<sura_msgs::srv::ClearControllerIntents::Request>();
      request->controller = controller_;
      // No stored future: the response callback removes pending requests.
      clear_client_->async_send_request(
        request,
        [](rclcpp::Client<sura_msgs::srv::ClearControllerIntents>::SharedFuture) {});
    }
    if (mode_request_id_ != 0 && mode_client_) {
      mode_client_->remove_pending_request(mode_request_id_);
      mode_request_id_ = 0;
    }
    mode_future_ = {};
    if (goal_ && goal_->is_active()) {
      auto result = std::make_shared<typename Action::Result>();
      result->success = success;
      result->message = message;
      if (goal_->is_canceling()) {goal_->canceled(result);} else if (success) {
        goal_->succeed(result);
      } else {goal_->abort(result);}
    }
    goal_.reset();
    // Execution is timer-based, so transitioning here cannot join its own worker.
    // Finish the transition before servicing the next lifecycle/action request.
    if (auto_deactivate && active()) {deactivate();}
  }

  bool canEdit(std::string & message)
  {
    if (!inactive()) {message = "Path editing requires inactive lifecycle state"; return false;}
    return true;
  }

  void configurePathServices()
  {
    add_service_ = create_service<sura_actions::srv::AddWaypointUsv>(
      topic("path_manager/add_waypoint"),
      [this](const std::shared_ptr<sura_actions::srv::AddWaypointUsv::Request> request,
      std::shared_ptr<sura_actions::srv::AddWaypointUsv::Response> response) {
        response->index = -1;
        if (!canEdit(response->message)) {return;}
        Pose2D point;
        point.x = request->x; point.y = request->y; point.theta = request->yaw;
        if (!finitePose(point)) {response->message = "Waypoint must be finite"; return;}
        point.theta = normalizeAngle(point.theta);
        auto points = editor_.points();
        response->index = static_cast<int32_t>(points.size());
        points.push_back(point);
        editor_.setPoints(std::move(points));
        response->success = true;
        response->message = "Waypoint added";
      });
    remove_service_ = create_service<std_srvs::srv::Trigger>(
      topic("path_manager/remove_last_waypoint"),
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!canEdit(response->message)) {return;}
        auto points = editor_.points();
        if (points.empty()) {response->message = "Path is empty"; return;}
        points.pop_back();
        editor_.setPoints(std::move(points));
        response->success = true;
        response->message = "Last waypoint removed";
      });
    clear_service_ = create_service<std_srvs::srv::Trigger>(
      topic("path_manager/clear_path"),
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!canEdit(response->message)) {return;}
        editor_.setPoints({});
        response->success = true;
        response->message = "Path cleared";
      });
    save_service_ = create_service<sura_actions::srv::PathFile>(
      topic("path_manager/save_path"),
      [this](const std::shared_ptr<sura_actions::srv::PathFile::Request> request,
      std::shared_ptr<sura_actions::srv::PathFile::Response> response) {
        if (!canEdit(response->message)) {return;}
        response->success = savePath(
          request->path_file, frame_,
          editor_.points(), response->message);
        if (response->success) {response->message = "Path saved";}
      });
    load_service_ = create_service<sura_actions::srv::PathFile>(
      topic("path_manager/load_path"),
      [this](const std::shared_ptr<sura_actions::srv::PathFile::Request> request,
      std::shared_ptr<sura_actions::srv::PathFile::Response> response) {
        if (!canEdit(response->message)) {return;}
        std::vector<Pose2D> points;
        response->success = loadPath(request->path_file, frame_, points, response->message);
        if (response->success) {
          editor_.setPoints(std::move(points)); response->message = "Path loaded";
        }
      });
  }

  void cleanupResources()
  {
    if (goal_ && rclcpp::ok(get_node_base_interface()->get_context())) {
      finish(false, "USV server shutting down", false);
    }
    goal_.reset();
    timer_.reset();
    action_server_.reset();
    navigator_sub_.reset();
    velocity_pub_.reset();
    mode_client_.reset();
    clear_client_.reset();
    add_service_.reset(); remove_service_.reset(); clear_service_.reset();
    save_service_.reset(); load_service_.reset();
    editor_.cleanup();
    navigation_received_ = false;
    navigation_valid_ = false;
  }

  std::string namespace_, frame_, controller_, requester_;
  int priority_{60};
  GuidanceLimits defaults_;
  double default_timeout_{120.0}, navigator_timeout_{2.0}, mode_timeout_{5.0};
  Pose2D current_;
  bool navigation_received_{false}, navigation_valid_{false};
  bool mode_requested_{false}, mode_ready_{false};
  int64_t mode_request_id_{0};
  size_t waypoint_{0};
  rclcpp::Time started_{0, 0, RCL_ROS_TIME}, navigation_stamp_{0, 0, RCL_ROS_TIME};
  ResolvedGoal execution_;
  PlanarEditor editor_;
  typename rclcpp_action::Server<Action>::SharedPtr action_server_;
  std::shared_ptr<Handle> goal_;
  rclcpp::Client<Mode>::SharedPtr mode_client_;
  rclcpp::Client<Mode>::SharedFuture mode_future_;
  rclcpp::Client<sura_msgs::srv::ClearControllerIntents>::SharedPtr clear_client_;
  rclcpp::Publisher<sura_msgs::msg::SuraVelocityCommand>::SharedPtr velocity_pub_;
  rclcpp::Subscription<sura_msgs::msg::Navigator>::SharedPtr navigator_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<sura_actions::srv::AddWaypointUsv>::SharedPtr add_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_service_, clear_service_;
  rclcpp::Service<sura_actions::srv::PathFile>::SharedPtr save_service_, load_service_;
};
}  // namespace sura_actions::usv
