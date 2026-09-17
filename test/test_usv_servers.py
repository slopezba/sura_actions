"""Exercise real action servers against a fake navigator and control manager."""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Pose2D, PoseStamped
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import Path as PathMsg
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger
from sura_actions.action import FollowPathUsv, GoToPoseUsv
from sura_actions.srv import AddWaypointUsv, PathFile, SetControlMode
from sura_msgs.msg import Navigator, SuraVelocityCommand
from sura_msgs.srv import ClearControllerIntents
from visualization_msgs.msg import (
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    InteractiveMarkerUpdate,
)


class Harness:
    def __init__(self, action_type, executable):
        self.action_type = action_type
        self.node = rclpy.create_node('usv_test_client')
        self.namespace = '/usv_test_' + str(os.getpid())
        self.name = executable
        self.navigation = Navigator()
        self.navigation.position.orientation.w = 1.0
        self.send_navigation = True
        self.commands = []
        self.modes = []
        self.clears = []
        self.feedback = []
        self.mode_success = True
        self.node.create_service(SetControlMode, self.namespace + '/control_manager/set_mode',
                                 self.set_mode)
        self.node.create_service(
            ClearControllerIntents,
            self.namespace + '/controller/arbitrator/clear_controller_intents', self.clear)
        self.pub = self.node.create_publisher(
            Navigator, self.namespace + '/navigator/navigation', 10)
        self.node.create_subscription(SuraVelocityCommand,
                                      self.namespace + '/controller/arbitrator/velocity',
                                      self.commands.append, 10)
        self.node.create_timer(0.02, self.publish_navigation)
        binary = Path(get_package_prefix('sura_actions')) / 'lib' / 'sura_actions' / executable
        self.log = tempfile.TemporaryFile(mode='w+')
        self.process = subprocess.Popen([
            str(binary), '--ros-args', '-p', 'robot_namespace:=' + self.namespace,
            '-p', 'navigator_timeout:=0.4', '-p', 'mode_request_timeout:=0.6',
            '-p', 'control_loop_rate:=50.0',
        ], stdout=self.log, stderr=subprocess.STDOUT)
        self.lifecycle = self.node.create_client(ChangeState, '/' + executable + '/change_state')
        self.state = self.node.create_client(GetState, '/' + executable + '/get_state')
        suffix = 'go_to_pose' if action_type is GoToPoseUsv else 'follow_path'
        self.client = ActionClient(self.node, action_type, self.namespace + '/actions/' + suffix)

    def set_mode(self, request, response):
        self.modes.append(request.mode)
        response.success = self.mode_success
        response.active_mode = request.mode
        response.message = 'test mode'
        return response

    def clear(self, request, response):
        self.clears.append(request.controller)
        response.success = True
        return response

    def publish_navigation(self):
        if self.send_navigation:
            self.pub.publish(self.navigation)

    def pose(self, x=0.0, y=0.0, yaw=0.0):
        self.navigation.position.position.x = float(x)
        self.navigation.position.position.y = float(y)
        self.navigation.position.orientation.z = math.sin(yaw / 2.0)
        self.navigation.position.orientation.w = math.cos(yaw / 2.0)

    def wait(self, predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            assert self.process.poll() is None, 'Server process exited'
        assert predicate(), 'Timed out waiting for ROS event'

    def future(self, future):
        self.wait(future.done)
        return future.result()

    def transition(self, transition):
        assert self.lifecycle.wait_for_service(timeout_sec=5.0)
        request = ChangeState.Request()
        request.transition.id = transition
        assert self.future(self.lifecycle.call_async(request)).success

    def activate(self):
        self.wait(lambda: self.state.service_is_ready())
        state = self.future(self.state.call_async(GetState.Request())).current_state.id
        if state == 1:
            self.transition(1)
        state = self.future(self.state.call_async(GetState.Request())).current_state.id
        if state == 2:
            self.transition(3)
        assert self.client.wait_for_server(timeout_sec=5.0)
        self.wait(lambda: self.pub.get_subscription_count() > 0)
        # Let the timer deliver fresh navigation before submitting goals.
        until = time.monotonic() + 0.1
        self.wait(lambda: time.monotonic() >= until)

    def goal(self, **kwargs):
        self.feedback.clear()
        goal = self.action_type.Goal(**kwargs)
        return self.future(self.client.send_goal_async(
            goal, feedback_callback=lambda message: self.feedback.append(message.feedback)))

    def result(self, handle, status):
        result = self.future(handle.get_result_async())
        assert result.status == status, result.result.message
        self.wait(lambda: bool(self.clears))
        self.wait(lambda: bool(self.commands) and
                  self.commands[-1].velocity.linear.x == 0.0 and
                  self.commands[-1].velocity.angular.z == 0.0)
        for command in self.commands:
            velocity = command.velocity
            assert velocity.linear.x >= 0
            assert (velocity.linear.y, velocity.linear.z,
                    velocity.angular.x, velocity.angular.y) == (0.0, 0.0, 0.0, 0.0)
        return result.result

    def close(self):
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.log.seek(0)
        print(self.log.read())
        self.log.close()
        self.client.destroy()
        self.node.destroy_node()


@pytest.fixture(params=[
    (GoToPoseUsv, 'go_to_pose_usv_lifecycle_action_node'),
    (FollowPathUsv, 'path_follower_usv_lifecycle_node'),
])
def server(request):
    rclpy.init()
    harness = None
    try:
        harness = Harness(*request.param)
        yield harness
    finally:
        if harness is not None:
            harness.close()
        rclpy.shutdown()


def goal_fields(server, point, **kwargs):
    if server.action_type is GoToPoseUsv:
        return dict(target_pose=point, **kwargs)
    return dict(path=[point], **kwargs)


def test_approach_final_yaw_and_origin(server):
    server.activate()
    handle = server.goal(**goal_fields(server, Pose2D(x=2.0, theta=1.0)))
    assert handle.accepted
    server.wait(lambda: any(c.velocity.linear.x > 0 for c in server.commands))
    assert server.modes[-1] == (5 if server.action_type is GoToPoseUsv else 1)
    server.pose(2.0)
    server.wait(lambda: any(f.state == 'aligning_final_yaw' for f in server.feedback))
    server.wait(lambda: server.commands[-1].velocity.linear.x == 0 and
                server.commands[-1].velocity.angular.z > 0)
    server.pose(2.0, yaw=1.0)
    assert server.result(handle, GoalStatus.STATUS_SUCCEEDED).success
    # An explicit all-zero target is valid and is not interpreted as a marker request.
    server.activate()
    server.pose()
    handle = server.goal(**goal_fields(server, Pose2D()))
    assert server.result(handle, GoalStatus.STATUS_SUCCEEDED).success


def test_cancel_timeout_navigation_and_deactivation(server):
    server.activate()
    fields = goal_fields(server, Pose2D(x=10.0))
    handle = server.goal(**fields)
    assert handle.accepted
    assert not server.goal(**fields).accepted
    server.future(handle.cancel_goal_async())
    server.result(handle, GoalStatus.STATUS_CANCELED)
    server.activate()
    handle = server.goal(**dict(fields, timeout=0.15))
    assert 'timed out' in server.result(handle, GoalStatus.STATUS_ABORTED).message
    server.activate()
    handle = server.goal(**fields)
    server.send_navigation = False
    assert 'Navigator' in server.result(handle, GoalStatus.STATUS_ABORTED).message
    server.send_navigation = True
    server.activate()
    handle = server.goal(**fields)
    server.transition(4)
    server.result(handle, GoalStatus.STATUS_ABORTED)
    assert not server.goal(**fields).accepted
    server.transition(2)
    server.send_navigation = False
    server.transition(1)
    server.transition(3)
    handle = server.goal(**fields)
    assert 'not received' in server.result(handle, GoalStatus.STATUS_ABORTED).message


def test_validation_and_mode_failure(server):
    server.activate()
    fields = goal_fields(server, Pose2D(x=float('nan')))
    assert not server.goal(**fields).accepted
    fields = goal_fields(server, Pose2D(x=10.0))
    assert not server.goal(**dict(fields, timeout=float('inf'))).accepted
    assert not server.goal(**dict(fields, max_forward_speed=-1.0)).accepted
    if server.action_type is FollowPathUsv:
        assert not server.goal(path=[]).accepted
    else:
        assert not server.goal(use_planned_pose=True).accepted
    server.mode_success = False
    handle = server.goal(**fields)
    assert 'Control mode failed' in server.result(handle, GoalStatus.STATUS_ABORTED).message


def test_path_editor_and_intermediate_yaw(server, tmp_path):
    if server.action_type is not FollowPathUsv:
        return
    server.transition(1)
    add = server.node.create_client(
        AddWaypointUsv, server.namespace + '/path_manager/add_waypoint')
    save = server.node.create_client(PathFile, server.namespace + '/path_manager/save_path')
    assert add.wait_for_service(timeout_sec=3)
    assert server.future(add.call_async(AddWaypointUsv.Request(x=1.0, yaw=2.0))).success
    file = tmp_path / 'path.xml'
    assert server.future(save.call_async(PathFile.Request(path_file=str(file)))).success
    assert ' z=' not in file.read_text()
    server.activate()
    assert not server.future(add.call_async(AddWaypointUsv.Request())).success
    handle = server.goal(path=[Pose2D(theta=2.0), Pose2D(x=2.0, theta=1.0)])
    assert handle.accepted
    server.wait(lambda: any(f.current_waypoint_index == 1 for f in server.feedback))
    server.pose(2.0, yaw=1.0)
    assert server.result(handle, GoalStatus.STATUS_SUCCEEDED).success
    assert server.feedback[-1].progress == 1.0
    # Execute the saved editor path using the same public action endpoint.
    server.pose(1.0, yaw=2.0)
    handle = server.goal(use_saved_path=True, path_file=str(file))
    assert server.result(handle, GoalStatus.STATUS_SUCCEEDED).success


def test_planar_marker_and_editor_services(server, tmp_path):
    single = server.action_type is GoToPoseUsv
    base = server.namespace + ('/actions/go_to_pose' if single else '/path_manager')
    marker_updates = []
    marker_topic = base + ('/interactive_marker/update' if single else
                           '/interactive_markers/update')
    server.node.create_subscription(
        InteractiveMarkerUpdate, marker_topic, marker_updates.append, 10)
    server.transition(1)
    if not single:
        add = server.node.create_client(AddWaypointUsv, base + '/add_waypoint')
        assert add.wait_for_service(timeout_sec=3)
        assert server.future(add.call_async(AddWaypointUsv.Request())).success

    def marker_controls():
        expected_name = 'go_to_pose_goal' if single else 'waypoint_0'
        for update in marker_updates:
            for marker in update.markers:
                if marker.name == expected_name:
                    return {control.name: control for control in marker.controls}
        return None

    server.wait(lambda: marker_controls() is not None)
    controls = marker_controls()
    for name in ('move_xy', 'move_x', 'move_y'):
        assert controls[name].orientation_mode == InteractiveMarkerControl.INHERIT
    assert controls['rotate_yaw'].orientation_mode == InteractiveMarkerControl.FIXED
    previews = []
    server.node.create_subscription(
        PoseStamped if single else PathMsg, base + ('/target_pose' if single else '/path'),
        previews.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher = server.node.create_publisher(
        InteractiveMarkerFeedback,
        base + ('/interactive_marker/feedback' if single else '/interactive_markers/feedback'), 10)
    server.wait(lambda: publisher.get_subscription_count() > 0)
    feedback = InteractiveMarkerFeedback()
    feedback.header.frame_id = 'world_ned'
    feedback.client_id = 'usv_test'
    feedback.marker_name = 'go_to_pose_goal' if single else 'waypoint_0'
    feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
    feedback.pose.position.x = 2.0
    feedback.pose.position.y = 1.0
    feedback.pose.position.z = 99.0
    feedback.pose.orientation.z = math.sin(0.5)
    feedback.pose.orientation.w = math.cos(0.5)
    publisher.publish(feedback)

    def planned_pose():
        if not previews:
            return None
        if single:
            return previews[-1].pose
        return previews[-1].poses[0].pose if previews[-1].poses else None

    server.wait(lambda: planned_pose() is not None and planned_pose().position.x == 2.0)
    assert planned_pose().position.z == 0.0
    assert planned_pose().orientation.x == planned_pose().orientation.y == 0.0
    server.pose(2.0, 1.0, 1.0)
    if single:
        server.activate()
        handle = server.goal(use_planned_pose=True)
        assert server.result(handle, GoalStatus.STATUS_SUCCEEDED).success
    else:
        file = tmp_path / 'auv_compatible.xml'
        file.write_text("<path frame_id='world_ned'><waypoint index='0' "
                        "x='2' y='1' z='-5' yaw='1'/></path>")
        load = server.node.create_client(PathFile, base + '/load_path')
        assert server.future(load.call_async(PathFile.Request(path_file=str(file)))).success
        remove = server.node.create_client(Trigger, base + '/remove_last_waypoint')
        assert server.future(remove.call_async(Trigger.Request())).success
        assert not server.future(remove.call_async(Trigger.Request())).success
        clear = server.node.create_client(Trigger, base + '/clear_path')
        assert server.future(clear.call_async(Trigger.Request())).success
        server.activate()
        handle = server.goal(use_saved_path=True, path_file=str(file))
        assert server.result(handle, GoalStatus.STATUS_SUCCEEDED).success
