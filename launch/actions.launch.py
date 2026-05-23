import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("sura_actions")
    default_control_modes_file = os.path.join(
        package_share,
        "config",
        "control_modes.yaml",
    )
    default_go_to_pose_non_holonomic_config_file = os.path.join(
        package_share,
        "config",
        "go_to_pose_non_holonomic.yaml",
    )
    default_path_manager_config_file = os.path.join(
        package_share,
        "config",
        "path_manager.yaml",
    )

    robot_namespace = LaunchConfiguration("robot_namespace")
    control_modes_file = LaunchConfiguration("control_modes_file")
    go_to_pose_non_holonomic_config_file = LaunchConfiguration(
        "go_to_pose_non_holonomic_config_file"
    )
    path_manager_config_file = LaunchConfiguration("path_manager_config_file")
    default_depth_tolerance = LaunchConfiguration("default_depth_tolerance")
    default_timeout = LaunchConfiguration("default_timeout")
    navigator_timeout = LaunchConfiguration("navigator_timeout")
    mode_request_timeout = LaunchConfiguration("mode_request_timeout")
    setpoint_publish_rate = LaunchConfiguration("setpoint_publish_rate")
    control_loop_rate = LaunchConfiguration("control_loop_rate")
    misalignment_slowdown_yaw = LaunchConfiguration("misalignment_slowdown_yaw")

    namespace_prefix = PythonExpression(["'/' + '", robot_namespace, "'.strip('/')"])
    controller_manager = PythonExpression(
        ["'", namespace_prefix, "' + '/controller/controller_manager'"]
    )
    surface_action_name = PythonExpression(["'", namespace_prefix, "' + '/actions/surface'"])
    go_to_pose_non_holonomic_action_name = PythonExpression(
        ["'", namespace_prefix, "' + '/actions/go_to_pose_non_holonomic'"]
    )
    follow_path_action_name = PythonExpression(
        ["'", namespace_prefix, "' + '/actions/follow_path'"]
    )
    navigator_topic = PythonExpression(["'", namespace_prefix, "' + '/navigator/navigation'"])
    depth_feedforward_topic = PythonExpression(
        ["'", namespace_prefix, "' + '/controller/depth_hold/feedforward'"]
    )
    body_velocity_command_topic = PythonExpression(
        ["'", namespace_prefix, "' + '/controller/body_velocity/setpoint'"]
    )
    depth_setpoint_topic = PythonExpression(
        ["'", namespace_prefix, "' + '/controller/depth_hold/set_point'"]
    )
    target_pose_topic = PythonExpression(
        ["'", namespace_prefix, "' + '/actions/go_to_pose_non_holonomic/target_pose'"]
    )
    path_manager_path_topic = PythonExpression(
        ["'", namespace_prefix, "' + '/path_manager/path'"]
    )
    path_manager_markers_topic = PythonExpression(
        ["'", namespace_prefix, "' + '/path_manager/markers'"]
    )
    path_manager_interactive_markers = PythonExpression(
        ["'", namespace_prefix, "' + '/path_manager/interactive_markers'"]
    )
    set_control_mode_service = PythonExpression(
        ["'", namespace_prefix, "' + '/control_manager/set_mode'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="sura"),
            DeclareLaunchArgument(
                "control_modes_file",
                default_value=default_control_modes_file,
            ),
            DeclareLaunchArgument(
                "go_to_pose_non_holonomic_config_file",
                default_value=default_go_to_pose_non_holonomic_config_file,
            ),
            DeclareLaunchArgument(
                "path_manager_config_file",
                default_value=default_path_manager_config_file,
            ),
            DeclareLaunchArgument("default_depth_tolerance", default_value="0.10"),
            DeclareLaunchArgument("default_timeout", default_value="30.0"),
            DeclareLaunchArgument("navigator_timeout", default_value="2.0"),
            DeclareLaunchArgument("mode_request_timeout", default_value="5.0"),
            DeclareLaunchArgument("setpoint_publish_rate", default_value="10.0"),
            DeclareLaunchArgument("control_loop_rate", default_value="15.0"),
            DeclareLaunchArgument("misalignment_slowdown_yaw", default_value="1.0"),
            Node(
                package="sura_actions",
                executable="sura_control_manager_node",
                name="sura_control_manager_node",
                output="screen",
                parameters=[
                    control_modes_file,
                    {
                        "robot_namespace": robot_namespace,
                        "controller_manager": controller_manager,
                    },
                ],
            ),
            Node(
                package="sura_actions",
                executable="surface_action_node",
                name="surface_action_node",
                output="screen",
                parameters=[
                    {
                        "robot_namespace": robot_namespace,
                        "action_name": surface_action_name,
                        "navigator_topic": navigator_topic,
                        "depth_feedforward_topic": depth_feedforward_topic,
                        "set_control_mode_service": set_control_mode_service,
                        "default_depth_tolerance": ParameterValue(
                            default_depth_tolerance,
                            value_type=float,
                        ),
                        "default_timeout": ParameterValue(
                            default_timeout,
                            value_type=float,
                        ),
                        "navigator_timeout": ParameterValue(
                            navigator_timeout,
                            value_type=float,
                        ),
                        "mode_request_timeout": ParameterValue(
                            mode_request_timeout,
                            value_type=float,
                        ),
                        "setpoint_publish_rate": ParameterValue(
                            setpoint_publish_rate,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="sura_actions",
                executable="go_to_pose_non_holonomic_action_node",
                name="go_to_pose_non_holonomic_action_node",
                output="screen",
                parameters=[
                    go_to_pose_non_holonomic_config_file,
                    {
                        "robot_namespace": robot_namespace,
                        "action_name": go_to_pose_non_holonomic_action_name,
                        "navigator_topic": navigator_topic,
                        "body_velocity_command_topic": body_velocity_command_topic,
                        "depth_setpoint_topic": depth_setpoint_topic,
                        "target_pose_topic": target_pose_topic,
                        "set_control_mode_service": set_control_mode_service,
                        "default_depth_tolerance": ParameterValue(
                            default_depth_tolerance,
                            value_type=float,
                        ),
                        "navigator_timeout": ParameterValue(
                            navigator_timeout,
                            value_type=float,
                        ),
                        "mode_request_timeout": ParameterValue(
                            mode_request_timeout,
                            value_type=float,
                        ),
                        "control_loop_rate": ParameterValue(
                            control_loop_rate,
                            value_type=float,
                        ),
                        "misalignment_slowdown_yaw": ParameterValue(
                            misalignment_slowdown_yaw,
                            value_type=float,
                        ),
                    }
                ],
            ),
            LifecycleNode(
                package="sura_actions",
                executable="path_manager_lifecycle_node",
                name="path_manager_lifecycle_node",
                namespace="",
                output="screen",
                parameters=[
                    path_manager_config_file,
                    {
                        "robot_namespace": robot_namespace,
                        "action_name": follow_path_action_name,
                        "navigator_topic": navigator_topic,
                        "body_velocity_command_topic": body_velocity_command_topic,
                        "depth_setpoint_topic": depth_setpoint_topic,
                        "path_topic": path_manager_path_topic,
                        "markers_topic": path_manager_markers_topic,
                        "interactive_marker_namespace": path_manager_interactive_markers,
                        "set_control_mode_service": set_control_mode_service,
                    },
                ],
            ),
        ]
    )
