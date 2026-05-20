cmake_minimum_required(VERSION 3.5)
project(odometry2)

# Default to C++17 (recomendado para ROS 2 Humble/Iron/Jazzy por tf2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Encontrar paquetes
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

# 2. Definir dependencias comunes para evitar repetición
set(dependencies 
  "rclcpp" 
  "std_msgs" 
  "sensor_msgs" 
  "nav_msgs" 
  "geometry_msgs" 
  "tf2" 
  "tf2_ros" 
  "std_srvs"
)

# 3. Configurar biblioteca de encoders
add_library(encoder STATIC src/encoder.cpp)
target_include_directories(encoder PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(encoder ${dependencies})

# 4. Nodo original (odometry2)
add_executable(odometry2 src/odometry2.cpp)
target_link_libraries(odometry2 encoder)
ament_target_dependencies(odometry2 ${dependencies})

# 5. Nodo IMU (Witmotion)
add_executable(imu src/imu.cpp)
ament_target_dependencies(imu ${dependencies})

# 6. Instalación
install(TARGETS 
  encoder
  odometry2
  imu
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()