cmake_minimum_required(VERSION 3.8)
project(sura_actions)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sura_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Surface.action"
  "action/GoToDepth.action"
  "action/GoToPose.action"
  "action/FollowPath.action"
  "action/GoToPoseUsv.action"
  "action/FollowPathUsv.action"
  "srv/AddWaypoint.srv"
  "srv/AddWaypointUsv.srv"
  "srv/PathFile.srv"
  "srv/SetControlMode.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(surface_action_node
  src/auv/surface_action_server.cpp
)
target_include_directories(surface_action_node
  PRIVATE
    include
)
ament_target_dependencies(surface_action_node
  rclcpp
  rclcpp_action
  sura_msgs
)
target_link_libraries(surface_action_node
  "${cpp_typesupport_target}"
)

add_executable(go_to_depth_lifecycle_action_node
  src/auv/go_to_depth_lifecycle_action_server.cpp
)
target_include_directories(go_to_depth_lifecycle_action_node
  PRIVATE
    include
)
ament_target_dependencies(go_to_depth_lifecycle_action_node
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sura_msgs
)
target_link_libraries(go_to_depth_lifecycle_action_node
  "${cpp_typesupport_target}"
)

add_executable(sura_control_manager_node
  src/sura_control_manager.cpp
)
target_include_directories(sura_control_manager_node
  PRIVATE
    include
)
ament_target_dependencies(sura_control_manager_node
  controller_manager_msgs
  rclcpp
  sura_msgs
)
target_link_libraries(sura_control_manager_node
  "${cpp_typesupport_target}"
)

add_executable(go_to_pose_lifecycle_action_node
  src/auv/go_to_pose_lifecycle_action_server.cpp
)
target_include_directories(go_to_pose_lifecycle_action_node
  PRIVATE
    include
)
ament_target_dependencies(go_to_pose_lifecycle_action_node
  geometry_msgs
  interactive_markers
  lifecycle_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sura_msgs
  visualization_msgs
)
target_link_libraries(go_to_pose_lifecycle_action_node
  "${cpp_typesupport_target}"
)

add_executable(path_manager_lifecycle_node
  src/auv/path_manager_lifecycle_node.cpp
)
target_include_directories(path_manager_lifecycle_node
  PRIVATE
    include
)
ament_target_dependencies(path_manager_lifecycle_node
  geometry_msgs
  interactive_markers
  lifecycle_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_srvs
  sura_msgs
  visualization_msgs
)
target_link_libraries(path_manager_lifecycle_node
  "${cpp_typesupport_target}"
  PkgConfig::TINYXML2
)

add_executable(go_to_pose_usv_lifecycle_action_node
  src/usv/go_to_pose_lifecycle_action_server.cpp)
add_executable(path_follower_usv_lifecycle_node
  src/usv/path_follower_lifecycle_server.cpp)
foreach(target go_to_pose_usv_lifecycle_action_node path_follower_usv_lifecycle_node)
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    geometry_msgs interactive_markers lifecycle_msgs nav_msgs rclcpp
    rclcpp_action rclcpp_lifecycle std_srvs sura_msgs visualization_msgs)
  target_link_libraries(${target} "${cpp_typesupport_target}" PkgConfig::TINYXML2)
endforeach()

install(
  TARGETS
    go_to_depth_lifecycle_action_node
    go_to_pose_lifecycle_action_node
    path_manager_lifecycle_node
    surface_action_node
    sura_control_manager_node
    go_to_pose_usv_lifecycle_action_node
    path_follower_usv_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_planar_guidance test/test_planar_guidance.cpp)
  target_include_directories(test_planar_guidance PRIVATE include)
  ament_target_dependencies(test_planar_guidance geometry_msgs)
  target_link_libraries(test_planar_guidance PkgConfig::TINYXML2)
  ament_add_pytest_test(test_usv_servers test/test_usv_servers.py TIMEOUT 120
    APPEND_ENV ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1)
  ament_add_pytest_test(test_actions_launch test/test_actions_launch.py
    APPEND_ENV PYTHONDONTWRITEBYTECODE=1)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
