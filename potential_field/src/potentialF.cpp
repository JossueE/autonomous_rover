#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

using std::placeholders::_1;

class PotentialField : public rclcpp::Node {
public:
  PotentialField() : Node("potential_field_node") {
    goal_x = this->declare_parameter<double>("initial_goal_x", 0.0);
    goal_y = this->declare_parameter<double>("initial_goal_y", 0.0);
    has_goal = this->declare_parameter<bool>("use_initial_goal", false);
    const auto goal_topic =
        this->declare_parameter<std::string>("goal_topic", "goal_pose");

    // Create Subscriber to the odometry
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&PotentialField::odom_callback, this, _1));
    // Create Subscriber to LiDAR
    sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&PotentialField::scan_callback, this, _1));
    // Create Subscriber to dynamic goals
    sub_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10, std::bind(&PotentialField::goal_callback, this, _1));

    // Create Publisher in command control
    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    // Create Publihser of the attraction vector
    att_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "attraction_vector", 1);
    // Create Publihser of the Replusion vector
    rep_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "repulsion_vector", 1);
    // Create Publihser of the Final vector
    fin_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "Final_Vector", 1);

    this->declare_parameter<double>("goal_tolerance", goal_tolerance);
    this->declare_parameter<double>("k_ang", k_ang);
    this->declare_parameter<double>("max_w", max_w);
    this->declare_parameter<double>("max_v", max_v);
    this->declare_parameter<double>("vector_epsilon", vector_epsilon);
    const double control_frequency =
        this->declare_parameter<double>("control_frequency", 20.0);

    load_control_parameters();

    // Create a periodic timer to drive the control loop at a fixed rate,
    // decoupled from the LiDAR scan callback frequency.
    const auto period_ms = std::chrono::duration<double, std::milli>(
        1000.0 / control_frequency);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period_ms),
        std::bind(&PotentialField::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
                "Listening for goals on '%s'. Control frequency: %.1f Hz.",
                goal_topic.c_str(), control_frequency);
    if (has_goal) {
      RCLCPP_INFO(this->get_logger(), "Initial goal | x: %.3f | y: %.3f",
                  goal_x, goal_y);
    }
  }

  int controller() {
    // Initialize the command message
    geometry_msgs::msg::Twist direction;

    if (!has_odom) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for odometry before commanding the robot.");
      cmd_pub->publish(direction);
      return 0;
    }

    if (!has_goal) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for a goal on the goal topic.");
      cmd_pub->publish(direction);
      return 0;
    }

    load_control_parameters();

    const double dx_goal = goal_x - x_odom;
    const double dy_goal = goal_y - y_odom;
    const double distance_to_goal = std::hypot(dx_goal, dy_goal);

    // Check if the robot arrived at the destination using the real pose.
    if (distance_to_goal < goal_tolerance) {
      direction.linear.x = 0.0;
      direction.angular.z = 0.0;
      if (!goal_reached) {
        RCLCPP_INFO(this->get_logger(), "Target reached!");
        goal_reached = true;
      }
      cmd_pub->publish(direction);
      return 0;
    }

    goal_reached = false;

    // Calculate final vector components
    double x_final = V_attraction[0] + V_repulsion[0];
    double y_final = V_attraction[1] + V_repulsion[1];

    // Publish the final vector (for visualization/debugging)
    geometry_msgs::msg::PoseStamped finalvector =
        PublishVector(x_final, y_final);
    fin_pub->publish(finalvector);

    const double final_magnitude = std::hypot(x_final, y_final);
    if (final_magnitude < vector_epsilon) {
      direction.linear.x = 0.0;
      direction.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Potential field vector is near zero before reaching the goal.");
      cmd_pub->publish(direction);
      return 0;
    }

    // Calculate target angle using atan2 (handles all quadrants correctly)
    double angle = atan2(y_final, x_final);

    // Calculate the shortest angular difference (delta)
    double delta = normalize_angle(angle - theta);

    RCLCPP_DEBUG(this->get_logger(), "Angle to goal: %f", delta);

    // Control logic: keep turning toward the vector while moving less when
    // misaligned.
    direction.angular.z = std::clamp(k_ang * delta, -max_w, max_w);

    double alignment = std::max(0.0, std::cos(delta));
    direction.linear.x = max_v * alignment;

    // Publish the command
    cmd_pub->publish(direction);

    return 0;
  }

  void load_control_parameters() {
    this->get_parameter("goal_tolerance", goal_tolerance);
    this->get_parameter("k_ang", k_ang);
    this->get_parameter("max_w", max_w);
    this->get_parameter("max_v", max_v);
    this->get_parameter("vector_epsilon", vector_epsilon);
  }

  // Helper function to normalize angle to [-PI, PI]
  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }

  geometry_msgs::msg::PoseStamped PublishVector(double x, double y) {

    // Create the attraction vector to show in RVIZ
    geometry_msgs::msg::PoseStamped vector;
    // Set the frame id
    std::string id_frame = "odom";
    vector.header.frame_id = id_frame;
    // Set the time stamp (current time)
    vector.header.stamp = this->get_clock()->now();
    // set the position (it's always (0,0,0) as the reference frame is odom)
    vector.pose.position.x = x_odom;
    vector.pose.position.y = y_odom;
    vector.pose.position.z = 0;
    // Compute the theta angle
    double angle = std::atan2(y, x);

    // Init variable of quaterion
    tf2::Quaternion q;
    // Set the quaterion using euler angles
    q.setRPY(0, 0, angle);
    // Convert the geometry::quaternion message into pose::quaterion message
    vector.pose.orientation = tf2::toMsg(q);
    // Publish it

    return vector;
  }

  void ComputeAttraction(double x_goal, double y_goal) {
    // Log the goal position
    RCLCPP_DEBUG(this->get_logger(), "GOAL | x: %f | y: %f", x_goal, y_goal);

    // Compute the relative position of the goal from the current position
    double dx = x_goal - x_odom;
    double dy = y_goal - y_odom;

    // Compute the distance between the current position and the goal
    double distance = std::hypot(dx, dy);

    // Handle edge case: Avoid division by zero or extremely large forces
    if (distance < 1e-3) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Goal is too close to the current position; force set to zero.");
      V_attraction = {0.0f, 0.0f};
      return;
    }

    // Define the strength of the attraction
    constexpr double attraction_gain = 1.0;
    constexpr double max_attraction_force = 1.0;

    // Compute the magnitude of the attractive force.
    double F_attraction =
        std::min(attraction_gain * distance, max_attraction_force);

    // Compute the attraction vector
    V_attraction = {static_cast<float>(F_attraction * dx / distance),
                    static_cast<float>(F_attraction * dy / distance)};

    // Log the computed attraction force
    RCLCPP_DEBUG(this->get_logger(), "Attraction Force | x: %f | y: %f",
                 V_attraction[0], V_attraction[1]);

    // Visualize the attraction vector by publishing it
    geometry_msgs::msg::PoseStamped attraction =
        PublishVector(V_attraction[0], V_attraction[1]);
    att_pub->publish(attraction);
  }

private:
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto &frame_id = msg->header.frame_id;
    if (!frame_id.empty() && frame_id != "odom" && frame_id != "/odom") {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Ignoring goal in frame '%s'. Publish goals in the odom frame.",
          frame_id.c_str());
      return;
    }

    const double new_goal_x = msg->pose.position.x;
    const double new_goal_y = msg->pose.position.y;
    if (!std::isfinite(new_goal_x) || !std::isfinite(new_goal_y)) {
      RCLCPP_WARN(this->get_logger(),
                  "Ignoring goal with non-finite position.");
      return;
    }

    goal_x = new_goal_x;
    goal_y = new_goal_y;
    has_goal = true;
    goal_reached = false;

    // Attraction will be recomputed on the next odom callback / timer tick.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Goal updated | x: %.3f | y: %.3f", goal_x, goal_y);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract and set the current position from the Odometry message
    x_odom = msg->pose.pose.position.x;
    y_odom = msg->pose.pose.position.y;

    // Retrieve orientation as a quaternion
    const auto &orientation = msg->pose.pose.orientation;
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z,
                      orientation.w);

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Set theta (yaw) as the robot's current heading
    theta = yaw;
    has_odom = true;

    // Log current odometry data (uncomment for debugging)
    // RCLCPP_INFO(this->get_logger(), "Odometry | x: %f | y: %f | theta: %f",
    // x_odom, y_odom, theta);

    // Compute the attraction vector based on the updated odometry
    if (has_goal) {
      ComputeAttraction(goal_x, goal_y);
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
    if (!has_odom) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Ignoring scan until odometry is available.");
      return;
    }

    // Extract scan parameters
    float angle_min = _msg->angle_min;
    float angle_increment = _msg->angle_increment;
    const auto &scan = _msg->ranges;
    size_t num_readings = scan.size();

    // Initialize repulsion vector components
    float x_r = 0.0f;
    float y_r = 0.0f;
    int valid_points = 0;

    // Define constants
    constexpr float influence_radius = 1.0f;
    constexpr float repulsion_gain = 0.35f;
    constexpr float max_repulsion_force = 2.0f;
    const float sensor_min =
        std::isfinite(_msg->range_min) ? _msg->range_min : 0.0f;
    const float sensor_max =
        std::isfinite(_msg->range_max) ? _msg->range_max : influence_radius;
    const float min_distance = std::max(sensor_min, 0.05f);
    const float max_distance = std::min(sensor_max, influence_radius);

    // Process each laser scan reading
    for (size_t i = 0; i < num_readings; ++i) {
      float distance = scan[i];

      // Consider only valid readings inside the local obstacle radius
      if (std::isfinite(distance) && distance > min_distance &&
          distance < max_distance) {
        float repulsion_force =
            repulsion_gain * ((1.0f / distance) - (1.0f / influence_radius)) /
            (distance * distance);
        repulsion_force = std::min(repulsion_force, max_repulsion_force);
        float angle = angle_min + theta + angle_increment * i;

        // Calculate projection of the repulsion force onto x and y axes
        x_r -= repulsion_force * cos(angle);
        y_r -= repulsion_force * sin(angle);

        ++valid_points;
      }
    }

    // Handle cases with no valid scan points
    if (valid_points == 0) {
      V_repulsion = {0.0f, 0.0f};
    } else {
      x_r /= static_cast<float>(valid_points);
      y_r /= static_cast<float>(valid_points);
      // Update the repulsion vector
      V_repulsion = {x_r, y_r};
    }

    // Log the repulsion vector for debugging (uncomment if needed)
    // RCLCPP_INFO(this->get_logger(), "Repulsion Vector | x: %f, y: %f",
    // V_repulsion[0], V_repulsion[1]);

    // Visualize the repulsion vector in RViz
    geometry_msgs::msg::PoseStamped repulsion =
        PublishVector(V_repulsion[0], V_repulsion[1]);
    rep_pub->publish(repulsion);
    // NOTE: controller() is driven by control_timer_, not by the scan callback.
  }

  // Timer that drives the control loop at a fixed rate
  rclcpp::TimerBase::SharedPtr control_timer_;
  // Periodic callback for the control loop
  void control_loop() { controller(); }

  // odom subsriber variable declaration
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
  // scan subsriber variable declaration
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
  // goal subscriber variable declaration
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal;
  // robot control publisher variable declaration
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
  // attraction vector publisher variable declaration
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr att_pub;
  // repulsion vector publisher variable declaration
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rep_pub;
  // repulsion vector publisher variable declaration
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fin_pub;
  // Declare position

  // x position of odometry
  double x_odom{0.0};
  // y position of odoemtry
  double y_odom{0.0};
  // Angle of the robot
  double theta{0.0};
  // Attraction vector
  std::array<float, 2> V_attraction{0.0f, 0.0f};
  // Replusion vector
  std::array<float, 2> V_repulsion{0.0f, 0.0f};
  //
  double goal_x{0.0};
  double goal_y{0.0};
  double goal_tolerance{0.1};
  double k_ang{1.0};
  double max_w{0.5};
  double max_v{0.22};
  double vector_epsilon{1e-6};
  bool has_goal{false};
  bool has_odom{false};
  bool goal_reached{false};
};

int main(int argc, char *argv[]) {
  // init node
  rclcpp::init(argc, argv);

  // init class
  auto node = std::make_shared<PotentialField>();
  rclcpp::spin(node);
  // shutdown once finished
  rclcpp::shutdown();
  // end
  return 0;
}
