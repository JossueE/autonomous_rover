#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rtabmap_msgs/msg/odom_info.hpp>

#include "rover_bt/msg/command.hpp"
#include "rover_bt/msg/rover_status.hpp"
#include "rover_bt/msg/sensor_health.hpp"
#include "rover_bt/srv/send_command.hpp"
#include "rover_bt/srv/save_location.hpp"
#include "rover_bt/action/navigate_to_goal.hpp"

#include "rover_bt/shared_context.hpp"
#include "rover_bt/command_arbitrator.hpp"
#include "rover_bt/location_registry.hpp"
#include "rover_bt/tts_client.hpp"

namespace rover_bt {

class RoverBTNode : public rclcpp::Node {
public:
  RoverBTNode();
  ~RoverBTNode() override = default;

private:
  void init_parameters();
  void init_ros_interfaces();
  void init_subsystems();
  void init_behavior_tree();

  // Callbacks
  void on_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_rtabmap_odom_info(const rtabmap_msgs::msg::OdomInfo::SharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_wheel_data(const std_msgs::msg::Float64::SharedPtr msg);
  void on_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void on_localization_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void on_trajectory(const nav_msgs::msg::Path::SharedPtr msg);
  void on_command(const rover_bt::msg::Command::SharedPtr msg);
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void on_person_status(const std_msgs::msg::String::SharedPtr msg);

  // Services
  void handle_send_command(
    const std::shared_ptr<rover_bt::srv::SendCommand::Request> request,
    std::shared_ptr<rover_bt::srv::SendCommand::Response> response);
  void handle_save_location(
    const std::shared_ptr<rover_bt::srv::SaveLocation::Request> request,
    std::shared_ptr<rover_bt::srv::SaveLocation::Response> response);

  // Timer Tick
  void tick_tree();

  // Periodic status publication (~1 Hz)
  void publish_status();

  // BT Components
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::shared_ptr<SharedContext> ctx_;

  // Subsystems
  std::unique_ptr<CommandArbitrator> arbitrator_;
  std::unique_ptr<LocationRegistry> location_registry_;
  std::unique_ptr<TTSClient> tts_;

  // ROS Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<rover_bt::msg::RoverStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr person_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr person_profile_pub_;

  // ROS Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rtabmap_msgs::msg::OdomInfo>::SharedPtr rtabmap_odom_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  // Both robots' wheel drivers publish wheel/left_data + wheel/right_data; a
  // message on either updates the motor/wheel liveness timestamp.
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_right_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<rover_bt::msg::Command>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr person_status_sub_;

  // ROS Services
  rclcpp::Service<rover_bt::srv::SendCommand>::SharedPtr send_command_srv_;
  rclcpp::Service<rover_bt::srv::SaveLocation>::SharedPtr save_location_srv_;

  // Timers
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Cached stale timeouts for sensor-health reporting
  double lidar_stale_timeout_{5.0};
  double camera_stale_timeout_{5.0};
  double odom_stale_timeout_{3.0};
  double imu_stale_timeout_{5.0};
  double motor_stale_timeout_{5.0};
  bool monitor_lidar_{false};
  bool monitor_wheels_{true};

  // Previous coarse person-tracker state, for edge-detecting lost/found events.
  std::string last_person_state_{"DISABLED"};
};

}  // namespace rover_bt
