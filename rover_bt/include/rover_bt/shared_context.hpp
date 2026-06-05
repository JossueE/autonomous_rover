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
  // last_motor_time = last time EITHER wheel encoder topic was received.
  // Both robots (zlac706 / zlac8015d) publish wheel/left_data + wheel/right_data,
  // so this is the "wheels are reporting" liveness signal. The encoder *value*
  // is irrelevant here — we only care that messages keep arriving.
  std::atomic<double> last_motor_time{0.0};
  std::atomic<double> last_trajectory_time{0.0};

  // Time (seconds, same clock as sensor stamps) the node finished init. Used to
  // bound the sensor-health startup grace period so a sensor that NEVER comes up
  // eventually reports STALE instead of being treated as healthy forever.
  std::atomic<double> node_start_time{0.0};

  // Whether the lidar watchdog is active. Real hardware has no lidar (sim only),
  // so this defaults false and is enabled from the sim launch. When false,
  // CheckSensorHealth("lidar") short-circuits to SUCCESS and status reports it
  // as disabled rather than perpetually stale.
  std::atomic<bool> monitor_lidar{false};

  // Whether the wheel-encoder watchdog is active. Real hardware runs the wheel
  // drivers (wheel/left_data + wheel/right_data); Gazebo sim does not publish
  // them, so the sim launch disables this to keep the wheel e-stop gate from
  // firing once the sim robot starts moving.
  std::atomic<bool> monitor_wheels{true};

  // Last time the joystick was used with its deadman held. Drives automatic
  // TELEOP_JOYCON entry/exit (CheckJoyActive). 0.0 = never used.
  std::atomic<double> last_joy_active_time{0.0};

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
