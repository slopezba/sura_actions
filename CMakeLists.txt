cmake_minimum_required(VERSION 3.8)
project(sura_actions)

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
  "action/GoToPoseNonHolonomic.action"
  "action/FollowPath.action"
  "srv/AddWaypoint.srv"
  "srv/PathFile.srv"
  "srv/SetControlMode.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(surface_action_node
  src/surface_action_server.cpp
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
)
target_link_libraries(sura_control_manager_node
  "${cpp_typesupport_target}"
)

add_executable(go_to_pose_non_holonomic_action_node
  src/go_to_pose_non_holonomic_action_server.cpp
)
target_include_directories(go_to_pose_non_holonomic_action_node
  PRIVATE
    include
)
ament_target_dependencies(go_to_pose_non_holonomic_action_node
  geometry_msgs
  rclcpp
  rclcpp_action
  sura_msgs
)
target_link_libraries(go_to_pose_non_holonomic_action_node
  "${cpp_typesupport_target}"
)

add_executable(path_manager_lifecycle_node
  src/path_manager_lifecycle_node.cpp
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

install(
  TARGETS
    go_to_pose_non_holonomic_action_node
    path_manager_lifecycle_node
    surface_action_node
    sura_control_manager_node
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
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
