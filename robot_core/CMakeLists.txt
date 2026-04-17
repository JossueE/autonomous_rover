cmake_minimum_required(VERSION 3.8)
project(robot_core)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(amcl_robot_pose src/amcl_robot_pose.cpp)
ament_target_dependencies(amcl_robot_pose
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  amcl_robot_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
