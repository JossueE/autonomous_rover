cmake_minimum_required(VERSION 3.14)
project(rover_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── Dependencies ─────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ─── Message / Service / Action generation ────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorHealth.msg"
  "msg/Command.msg"
  "msg/RoverStatus.msg"
  "srv/SendCommand.srv"
  "srv/SaveLocation.srv"
  "action/NavigateToGoal.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# ─── Source files ─────────────────────────────────────────────────────
set(BT_CONDITION_SOURCES
  src/nodes/conditions/check_mode.cpp
  src/nodes/conditions/check_command.cpp
  src/nodes/conditions/check_sensor_health.cpp
  src/nodes/conditions/check_nav_status.cpp
  src/nodes/conditions/check_flag.cpp
  src/nodes/conditions/check_joy_active.cpp
  src/nodes/conditions/check_person_event.cpp
)

set(BT_ACTION_SOURCES
  src/nodes/actions/navigate_to_goal.cpp
  src/nodes/actions/zero_twist.cpp
  src/nodes/actions/move_rover.cpp
  src/nodes/actions/speak.cpp
  src/nodes/actions/set_mode.cpp
  src/nodes/actions/set_flag.cpp
  src/nodes/actions/clear_command.cpp
  src/nodes/actions/start_mapping.cpp
  src/nodes/actions/stop_mapping.cpp
  src/nodes/actions/request_replan.cpp
  src/nodes/actions/publish_status.cpp
  src/nodes/actions/increment_patrol_index.cpp
  src/nodes/actions/save_location.cpp
  src/nodes/actions/process_command.cpp
  src/nodes/actions/set_person_profile.cpp
)

set(CORE_SOURCES
  src/rover_bt_node.cpp
  src/command_arbitrator.cpp
  src/location_registry.cpp
  src/tts_client.cpp
)

# ─── Main executable ─────────────────────────────────────────────────
add_executable(rover_bt_node
  ${CORE_SOURCES}
  ${BT_CONDITION_SOURCES}
  ${BT_ACTION_SOURCES}
)

target_include_directories(rover_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(ROVER_BT_DEPS
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  behaviortree_cpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  rtabmap_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

ament_target_dependencies(rover_bt_node ${ROVER_BT_DEPS})

# Link to our own generated message/service/action types
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(rover_bt_node "${cpp_typesupport_target}")

# ─── Install ─────────────────────────────────────────────────────────
install(TARGETS rover_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/voice_command_node.py
  scripts/install_voice_assets.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/stt
  scripts/tts
  scripts/config
  scripts/utils
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config trees
  DESTINATION share/${PROJECT_NAME}
)

# ─── Testing ─────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
