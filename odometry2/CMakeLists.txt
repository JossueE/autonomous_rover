cmake_minimum_required(VERSION 3.5)
project(odometry2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies "rclcpp" "std_msgs" "nav_msgs" "tf2" "tf2_ros" "std_srvs")

# Define include directories
set(INCLUDE_DIRS include)

## IMPORTANT include dirs
include_directories(${ament_cmake_INCLUDE_DIRS} ${INCLUDE_DIRS})


## Declare a C++ library
set(LIBS_FILES src/encoder.cpp)

add_library(${PROJECT_NAME}_LIBRARIES ${LIBS_FILES} ${INCLUDE_DIRS})
add_library(encoder STATIC src/encoder.cpp)
target_include_directories(encoder PUBLIC include)

add_executable(odometry2 src/odometry2.cpp ${LIBS_FILES})
ament_target_dependencies(odometry2 ${dependencies})
target_link_libraries(odometry2 encoder) # link with the encoder library

install(TARGETS 
  odometry2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY src include
  DESTINATION share/${PROJECT_NAME})

ament_package()
