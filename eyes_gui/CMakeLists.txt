cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()
project(eyes_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rover_bt REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

find_package(map_msgs QUIET)
find_package(rtabmap_msgs QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFaceState.srv"
)

set(EXECUTABLE_NAME robot_face_debug_ui)

add_executable(${EXECUTABLE_NAME}
  include/eyes_gui/debug_view.hpp
  include/eyes_gui/face_state_node.hpp
  include/eyes_gui/face_view.hpp
  include/eyes_gui/idle_event_filter.hpp
  include/eyes_gui/main_window.hpp
  src/debug_view.cpp
  src/face_state_node.cpp
  src/face_view.cpp
  src/idle_event_filter.cpp
  src/main.cpp
  src/main_window.cpp
)

add_executable(face_emotion_controller
  src/face_emotion_controller.cpp
)

target_include_directories(${EXECUTABLE_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(face_emotion_controller
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(${EXECUTABLE_NAME}
  ament_index_cpp
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  sensor_msgs
  std_msgs
  visualization_msgs
)

ament_target_dependencies(face_emotion_controller
  nav_msgs
  rclcpp
  rover_bt
)

if(map_msgs_FOUND)
  ament_target_dependencies(${EXECUTABLE_NAME} map_msgs)
  target_compile_definitions(${EXECUTABLE_NAME} PRIVATE HAVE_MAP_MSGS)
endif()

if(rtabmap_msgs_FOUND)
  ament_target_dependencies(${EXECUTABLE_NAME} rtabmap_msgs)
  target_compile_definitions(${EXECUTABLE_NAME} PRIVATE HAVE_RTABMAP_MSGS)
endif()

target_link_libraries(${EXECUTABLE_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(${EXECUTABLE_NAME} "${cpp_typesupport_target}")
target_link_libraries(face_emotion_controller "${cpp_typesupport_target}")

install(TARGETS
  ${EXECUTABLE_NAME}
  face_emotion_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY images
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
