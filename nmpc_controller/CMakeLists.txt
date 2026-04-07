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

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)

# CasADi through pkg-config
# If this fails in your machine, check whether the pkg-config module is named
# 'casadi' or if CasADi was installed without pkg-config support.
pkg_check_modules(CASADI REQUIRED IMPORTED_TARGET casadi)

set(ROS2_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Include dirs
set(INCLUDE_DIRS include)
include_directories(${INCLUDE_DIRS})

# -----------------------------
# Core library (no ROS2 logic)
# -----------------------------
add_library(nmpc_controller_core STATIC
  src/nmpc_controller.cpp
)

target_include_directories(nmpc_controller_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nmpc_controller_core
  PUBLIC
    PkgConfig::CASADI
)

if(CMAKE_DEBUG_FLAG)
  target_compile_definitions(nmpc_controller_core PUBLIC CMAKE_DEBUG_FLAG)
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
  PRIVATE
    nmpc_controller_core
)

if(CMAKE_DEBUG_FLAG)
  target_compile_definitions(nmpc_controller_node PRIVATE CMAKE_DEBUG_FLAG)
endif()

# -----------------------------
# Install
# -----------------------------
install(TARGETS
  nmpc_controller_core
  nmpc_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Optional, only if you want to keep sources in share like your other package
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