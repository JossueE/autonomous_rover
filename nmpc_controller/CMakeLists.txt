cmake_minimum_required(VERSION 3.8)
project(nmpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Debug flag definition
if(CMAKE_DEBUG_FLAG)
  message(STATUS "******* CMAKE_DEBUG_FLAG: ON")
else()
  message(WARNING "******* CMAKE_DEBUG_FLAG is unset/empty. Building without debug macro")
endif()

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(ROS2_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# -----------------------------
# CasADi manual lookup
# -----------------------------
find_path(CASADI_INCLUDE_DIR
  NAMES casadi/casadi.hpp
  PATHS
    /usr/include
    /usr/local/include
)

find_library(CASADI_LIBRARY
  NAMES casadi
  PATHS
    /usr/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib64
)

if(NOT CASADI_INCLUDE_DIR OR NOT CASADI_LIBRARY)
  message(FATAL_ERROR
    "CasADi not found.\n"
    "Set CASADI_INCLUDE_DIR and CASADI_LIBRARY manually, for example:\n"
    "colcon build --packages-select nmpc_controller --cmake-args "
    "-DCASADI_INCLUDE_DIR=/usr/local/include "
    "-DCASADI_LIBRARY=/usr/local/lib/libcasadi.so")
endif()

message(STATUS "CASADI_INCLUDE_DIR = ${CASADI_INCLUDE_DIR}")
message(STATUS "CASADI_LIBRARY     = ${CASADI_LIBRARY}")

# -----------------------------
# Core library (no ROS2 logic)
# -----------------------------
add_library(nmpc_controller
  STATIC
  src/nmpc_controller/nmpc_controller.cpp
)

target_include_directories(nmpc_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CASADI_INCLUDE_DIR}
)

target_link_libraries(nmpc_controller
  ${CASADI_LIBRARY}
)

if(CMAKE_DEBUG_FLAG)
  target_compile_definitions(nmpc_controller PUBLIC NMPC_DEBUG)
endif()

# -----------------------------
# ROS2 node executable
# -----------------------------
add_executable(nmpc_controller_node
  src/nmpc_controller_node.cpp
)

ament_target_dependencies(nmpc_controller_node
  ${ROS2_DEPENDENCIES}
)

target_link_libraries(nmpc_controller_node
  nmpc_controller
  tf2::tf2
)

if(CMAKE_DEBUG_FLAG)
  target_compile_definitions(nmpc_controller_node PRIVATE NMPC_DEBUG)
endif()

# -----------------------------
# Install
# -----------------------------
install(TARGETS
  nmpc_controller
  nmpc_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/path_drawer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
# Testing / lint
# -----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
