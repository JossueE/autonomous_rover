cmake_minimum_required(VERSION 3.8)
project(zlac706_driver2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs)
find_package(PkgConfig REQUIRED)

set(dependencies "rclcpp" "std_msgs" "geometry_msgs") # Dependencies for node

# Define include directories
set(INCLUDE_DIRS include)

## IMPORTANT include dirs
include_directories(${ament_cmake_INCLUDE_DIRS} ${INCLUDE_DIRS})

## Declare a C++ library
set(LIBS_FILES src/zlac706_driver/zlac706_driver.cpp src/zlac706_driver/AsyncSerial.cpp src/zlac706_driver/wheels_writer.cpp)


add_library(${PROJECT_NAME}_LIBRARIES ${LIBS_FILES} ${INCLUDE_DIRS})
add_library(zlac706_driver STATIC ${LIBS_FILES} $<$<CONFIG:Debug>:DEBUG>) #?????????? SOLO POR DEBUG MODE??
target_include_directories(zlac706_driver PUBLIC include)

add_executable(wheels_driver src/wheels_driver.cpp ${LIBS_FILES})
ament_target_dependencies(wheels_driver ${dependencies})
target_link_libraries(wheels_driver zlac706_driver) # link with the encoder library

# No ROS 2 node, but compiled and run as a one
# add_executable(test_driver src/zlac8015d_driver/test_driver.cpp)
# target_link_libraries(test_driver zlac8015d_driver) #PkgConfig::MODBUS
# message(" ****** Test code without ROS 2 ****** ")


install(TARGETS
  wheels_driver   # ROS 2 node
  #test_driver     # No ROS 2 node, but compiled and run as a one
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY src include
  DESTINATION share/${PROJECT_NAME})

ament_package()