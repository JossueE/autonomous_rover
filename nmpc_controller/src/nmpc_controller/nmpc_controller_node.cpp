#include "nmpc_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double wrapToPi(double angle)
{
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

double clampValue(const double value, const double low, const double high)
{
  return std::max(low, std::min(value, high));
}

bool isFiniteQuaternion(
  const geometry_msgs::msg::Quaternion & q)
{
  return std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w);
}
}  // namespace

class NMPCControllerNode : public rclcpp::Node, public NMPCController
{
public:
  NMPCControllerNode()
  : rclcpp::Node("nmpc_controller_node"),
    NMPCController(ControllerConfig{}),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {

    // ========================== Declare parameters ==========================
    this->declare_parameter<double>("h", 0.2);
    this->declare_parameter<int>("N", 20);
    this->declare_parameter<double>("L", 0.633);
    this->declare_parameter<double>("v_max", 0.8);
    this->declare_parameter<double>("a_max", 0.5);
    this->declare_parameter<double>("lambda_1", 0.25);
    this->declare_parameter<double>("d_safe", 0.8);
    this->declare_parameter<double>("voxel_size", 0.5);
    this->declare_parameter<double>("max_range", 3.5);

    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>(
      "costmap_topic", "/move_base/local_costmap/costmap");
    this->declare_parameter<std::string>("path_topic", "/nmpc_reference_path");
    
    // ========================== Load parameters ==========================
    config_.h = this->get_parameter("h").as_double();
    config_.N = this->get_parameter("N").as_int();
    config_.L = this->get_parameter("L").as_double();
    config_.v_max = this->get_parameter("v_max").as_double();
    config_.a_max = this->get_parameter("a_max").as_double();
    config_.lambda_1 = this->get_parameter("lambda_1").as_double();
    config_.d_safe = this->get_parameter("d_safe").as_double();
    config_.voxel_size = this->get_parameter("voxel_size").as_double();
    config_.max_range = this->get_parameter("max_range").as_double();

    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    costmap_topic_ = this->get_parameter("costmap_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();

    NMPCController::initialize();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10);

    occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&NMPCControllerNode::occupancyCallback, this, _1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic_,
      10,
      std::bind(&NMPCControllerNode::pathCallback, this, _1));

    const auto period = std::max(
      std::chrono::milliseconds(1),
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(config_.h)));

    timer_ = this->create_wall_timer(
      period,
      std::bind(&NMPCControllerNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "NMPCControllerNode initialized");
    RCLCPP_INFO(this->get_logger(), "map_frame: %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_vel_topic: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "costmap_topic: %s", costmap_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "path_topic: %s", path_topic_.c_str());
  }

private:

  void occupancyCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    occupancy_.origin_x = msg->info.origin.position.x;
    occupancy_.origin_y = msg->info.origin.position.y;
    occupancy_.resolution = msg->info.resolution;
    occupancy_.width = msg->info.width;
    occupancy_.height = msg->info.height;
    occupancy_.data = msg->data;
    has_occupancy_ = true;
  }

  void pathCallback(
    const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < static_cast<std::size_t>(config_.N)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Received path with %zu poses, but horizon N=%d. Ignoring.",
        msg->poses.size(), config_.N);
      return;
    }

    TrajectoryReference ref = buildReferenceFromPath(*msg);

    if (!ref.valid() || ref.size() < static_cast<std::size_t>(config_.N)) {
      RCLCPP_WARN(this->get_logger(), "Generated reference is invalid");
      return;
    }

    reference_ = std::move(ref);
    step_ = 0;
    active_ = true;
    has_reference_ = true;
    current_vr_ = 0.0;
    current_vl_ = 0.0;
    opt_states_cache_.clear();

    RCLCPP_INFO(
      this->get_logger(),
      "Reference loaded with %zu samples",
      reference_.size());
  }

  TrajectoryReference buildReferenceFromPath(
    const nav_msgs::msg::Path & path_msg) const
  {
    TrajectoryReference ref;
    const std::size_t n = path_msg.poses.size();

    ref.x.resize(n);
    ref.y.resize(n);
    ref.theta.resize(n);
    ref.vr.resize(n);
    ref.vl.resize(n);
    ref.ar.resize(n);
    ref.al.resize(n);

    if (n == 0) {
      return ref;
    }

    for (std::size_t i = 0; i < n; ++i) {
      ref.x[i] = path_msg.poses[i].pose.position.x;
      ref.y[i] = path_msg.poses[i].pose.position.y;
    }

    if (n == 1) {
      ref.theta[0] = 0.0;
      ref.vr[0] = 0.0;
      ref.vl[0] = 0.0;
      ref.ar[0] = 0.0;
      ref.al[0] = 0.0;
      return ref;
    }

    for (std::size_t i = 0; i < n - 1; ++i) {
      const double dx = ref.x[i + 1] - ref.x[i];
      const double dy = ref.y[i + 1] - ref.y[i];

      if (std::hypot(dx, dy) > 1e-9) {
        ref.theta[i] = std::atan2(dy, dx);
      } else if (isFiniteQuaternion(path_msg.poses[i].pose.orientation)) {
        ref.theta[i] = tf2::getYaw(path_msg.poses[i].pose.orientation);
      } else {
        ref.theta[i] = (i == 0) ? 0.0 : ref.theta[i - 1];
      }
    }
    ref.theta[n - 1] = ref.theta[n - 2];

    for (std::size_t i = 1; i < n; ++i) {
      const double dtheta = wrapToPi(ref.theta[i] - ref.theta[i - 1]);
      ref.theta[i] = ref.theta[i - 1] + dtheta;
    }

    std::vector<double> linear_v(n, 0.0);
    std::vector<double> angular_v(n, 0.0);

    for (std::size_t i = 0; i < n - 1; ++i) {
      const double dx = ref.x[i + 1] - ref.x[i];
      const double dy = ref.y[i + 1] - ref.y[i];
      const double ds = std::hypot(dx, dy);

      linear_v[i] = ds / config_.h;
      angular_v[i] = wrapToPi(ref.theta[i + 1] - ref.theta[i]) / config_.h;
    }

    linear_v[n - 1] = linear_v[n - 2];
    angular_v[n - 1] = angular_v[n - 2];

    for (std::size_t i = 0; i < n; ++i) {
      double vr = linear_v[i] + 0.5 * config_.L * angular_v[i];
      double vl = linear_v[i] - 0.5 * config_.L * angular_v[i];

      vr = clampValue(vr, -config_.v_max, config_.v_max);
      vl = clampValue(vl, -config_.v_max, config_.v_max);

      ref.vr[i] = vr;
      ref.vl[i] = vl;
    }

    ref.ar[0] = 0.0;
    ref.al[0] = 0.0;
    for (std::size_t i = 1; i < n; ++i) {
      ref.ar[i] = clampValue(
        (ref.vr[i] - ref.vr[i - 1]) / config_.h,
        -config_.a_max, config_.a_max);
      ref.al[i] = clampValue(
        (ref.vl[i] - ref.vl[i - 1]) / config_.h,
        -config_.a_max, config_.a_max);
    }

    return ref;
  }

  bool lookupCurrentState(RobotState & x0)
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_.lookupTransform(
        map_frame_,
        base_frame_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "TF lookup failed: %s",
        ex.what());
      return false;
    }

    x0.x = tf_msg.transform.translation.x;
    x0.y = tf_msg.transform.translation.y;
    x0.theta = tf2::getYaw(tf_msg.transform.rotation);
    x0.vr = current_vr_;
    x0.vl = current_vl_;

    return true;
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);

    current_vr_ = 0.0;
    current_vl_ = 0.0;
  }

  void controlLoop()
  {
    if (!active_ || !has_reference_) {
      return;
    }

    if (reference_.size() < static_cast<std::size_t>(config_.N)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Reference shorter than horizon");
      publishStop();
      active_ = false;
      return;
    }

    if (step_ + static_cast<std::size_t>(config_.N) > reference_.size()) {
      RCLCPP_INFO(this->get_logger(), "End of reference reached");
      publishStop();
      active_ = false;
      return;
    }

    RobotState x0;
    if (!lookupCurrentState(x0)) {
      publishStop();
      return;
    }

    const SolveResult result = solve(
      step_,
      reference_.size(),
      reference_,
      x0,
      occupancy_);

    if (!result.success) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "NMPC solve failed: %s",
        result.message.c_str());
      publishStop();
      return;
    }

    if (result.vr_horizon.empty() || result.vl_horizon.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "NMPC returned empty control horizon");
      publishStop();
      return;
    }

    const double vr_cmd = 0.5 * (x0.vr + result.vr_horizon.front());
    const double vl_cmd = 0.5 * (x0.vl + result.vl_horizon.front());

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = (vr_cmd + vl_cmd) / 2.0;
    cmd.angular.z = (vr_cmd - vl_cmd) / config_.L;
    cmd_vel_pub_->publish(cmd);

    current_vr_ = vr_cmd;
    current_vl_ = vl_cmd;

    RCLCPP_DEBUG(
      this->get_logger(),
      "step=%zu solver_time=%.6f data_time=%.6f obstacles=%zu v=%.4f w=%.4f",
      step_,
      result.solver_time,
      result.data_time,
      result.voxel_obstacles.size(),
      cmd.linear.x,
      cmd.angular.z);

    ++step_;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  OccupancyGridData occupancy_{};
  TrajectoryReference reference_{};

  std::string map_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  std::string costmap_topic_;
  std::string path_topic_;

  std::size_t step_{0};
  double current_vr_{0.0};
  double current_vl_{0.0};

  bool active_{false};
  bool has_reference_{false};
  bool has_occupancy_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMPCControllerNode>());
  rclcpp::shutdown();
  return 0;
}