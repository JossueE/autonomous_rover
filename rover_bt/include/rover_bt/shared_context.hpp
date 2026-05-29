#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace rover_bt {
class TTSClient;
class LocationRegistry;
class CommandArbitrator;

struct SharedContext {
  rclcpp::Node::SharedPtr node;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  // Shared subsystems (raw ptrs owned by RoverBTNode)
  TTSClient* tts = nullptr;
  LocationRegistry* location_registry = nullptr;
  CommandArbitrator* arbitrator = nullptr;

  // Sensor timestamps
  std::atomic<double> last_lidar_time{0.0};
  std::atomic<double> last_camera_time{0.0};
  std::atomic<double> last_odom_time{0.0};
  std::atomic<double> last_imu_time{0.0};
  std::atomic<double> last_motor_time{0.0};
  std::atomic<double> last_trajectory_time{0.0};

  // Robot pose
  std::atomic<double> robot_x{0.0};
  std::atomic<double> robot_y{0.0};
  std::atomic<double> robot_theta{0.0};
  std::atomic<double> robot_linear_vel{0.0};
  std::mutex pose_source_mutex;
  std::string pose_source{"odom"};  // "map" or "odom"

  // Patrol waypoints
  std::mutex waypoints_mutex;
  std::vector<std::string> patrol_waypoints;
};

}  // namespace rover_bt
