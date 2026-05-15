#include "nmpc_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double wrapToPi(double angle) {
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

double clampValue(const double value, const double low, const double high) {
  return std::max(low, std::min(value, high));
}

bool isFiniteQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w);
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
} // namespace

class NMPCControllerNode : public rclcpp::Node, public NMPCController {
public:
  NMPCControllerNode()
      : rclcpp::Node("nmpc_controller_node"),
        NMPCController(ControllerConfig{}), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    // NMPC discretization and differential-drive model parameters.
    this->declare_parameter<double>("h", 0.2);      // Sampling time [s].
    this->declare_parameter<int>("N", 10);          // Prediction horizon length.
    this->declare_parameter<double>("L", 0.160);    // Track width [m].
    this->declare_parameter<double>("v_max", 0.22); // Max wheel speed [m/s].
    this->declare_parameter<double>("a_max", 0.35); // Max wheel acceleration [m/s^2].

    // NMPC cost weights.
    this->declare_parameter<double>("lambda_1", 2.0);      // Smoothness weight for acceleration changes.
    this->declare_parameter<double>("lambda_theta", 0.25); // Heading tracking weight.
    this->declare_parameter<double>("lambda_v", 0.02);     // Wheel-speed tracking weight.

    // Obstacle handling parameters.
    this->declare_parameter<double>("d_safe", 0.2);      // Minimum obstacle clearance [m].
    this->declare_parameter<double>("voxel_size", 0.5);  // Obstacle voxel size [m].
    this->declare_parameter<double>("max_range", 3.5);   // Obstacle search range [m].
    this->declare_parameter<int>("max_obstacles", 20);   // Fixed obstacle slots in the NLP.

    // ROS frame and topic configuration.
    this->declare_parameter<std::string>("map_frame", "map");         // Global planning / control frame.
    this->declare_parameter<std::string>("base_frame", "base_footprint");  // Robot body frame.
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel"); // Velocity command output topic.
    this->declare_parameter<std::string>("costmap_topic",
                                         "/move_base/local_costmap/costmap"); // Occupancy grid input topic.
    this->declare_parameter<std::string>("path_topic", "/drawn_plan");        // Path reference input topic.
    this->declare_parameter<double>("goal_tolerance", 0.05);                  // Goal acceptance radius [m].

    config_.h = this->get_parameter("h").as_double();
    config_.N = this->get_parameter("N").as_int();
    config_.L = this->get_parameter("L").as_double();
    config_.v_max = this->get_parameter("v_max").as_double();
    config_.a_max = this->get_parameter("a_max").as_double();
    config_.lambda_1      = this->get_parameter("lambda_1").as_double();
    config_.lambda_theta  = this->get_parameter("lambda_theta").as_double();
    config_.lambda_v      = this->get_parameter("lambda_v").as_double();
    config_.d_safe        = this->get_parameter("d_safe").as_double();
    config_.voxel_size    = this->get_parameter("voxel_size").as_double();
    config_.max_range     = this->get_parameter("max_range").as_double();
    config_.max_obstacles = this->get_parameter("max_obstacles").as_int();

    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    costmap_topic_ = this->get_parameter("costmap_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    NMPCController::initialize();

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic_, rclcpp::SensorDataQoS(),
        std::bind(&NMPCControllerNode::occupancyCallback, this, _1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_, 10,
        std::bind(&NMPCControllerNode::pathCallback, this, _1));

    const auto period = std::max(
        std::chrono::milliseconds(1),
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double, std::milli>(config_.h * 1000.0)));

    timer_ = this->create_wall_timer(
        period, std::bind(&NMPCControllerNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "NMPCControllerNode initialized");
    RCLCPP_INFO(this->get_logger(), "map_frame: %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_vel_topic: %s",
                cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "costmap_topic: %s",
                costmap_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "path_topic: %s", path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "goal_tolerance: %.3f m",
                goal_tolerance_);
  }

private:

  // Convert the discrete reference into cumulative arc length so the controller
  // can track progress along the path independently of the point density.
  std::vector<double>
  buildCumulativeDistances(const TrajectoryReference &ref) const {
    std::vector<double> distances(ref.size(), 0.0);

    for (std::size_t i = 1; i < ref.size(); ++i) {
      distances[i] = distances[i - 1] +
                     std::hypot(ref.x[i] - ref.x[i - 1],
                                ref.y[i] - ref.y[i - 1]);
    }

    return distances;
  }

  // Map a continuous progress value s [m] back to the first reference sample
  // whose cumulative distance is greater than or equal to s.
  std::size_t stepFromProgress(
      const std::vector<double> &distances,
      const double progress) const {
    if (distances.empty()) {
      return 0;
    }

    const auto it = std::lower_bound(distances.begin(), distances.end(),
                                     progress);
    if (it == distances.end()) {
      return distances.size() - 1;
    }

    return static_cast<std::size_t>(std::distance(distances.begin(), it));
  }

  // Project the robot position onto nearby path segments and estimate how far
  // along the current reference the robot has already progressed.
  double estimateProgressAlongReference(
      const RobotState &x0, const TrajectoryReference &ref,
      const std::vector<double> &distances,
      const std::size_t hint_idx) const {
    if (ref.empty() || distances.empty()) {
      return 0.0;
    }

    if (ref.size() == 1) {
      return 0.0;
    }

    const std::size_t anchor = std::min(hint_idx, ref.size() - 1);
    const std::size_t back_window = 2;
    const std::size_t forward_window =
        std::max<std::size_t>(static_cast<std::size_t>(config_.N) * 2U, 10U);
    const std::size_t first_seg =
        (anchor > back_window) ? anchor - back_window : 0;
    const std::size_t last_seg =
        std::min(ref.size() - 2, anchor + forward_window);

    double best_progress = distances[anchor];
    double best_d2 = std::numeric_limits<double>::infinity();

    for (std::size_t i = first_seg; i <= last_seg; ++i) {
      const double ax = ref.x[i];
      const double ay = ref.y[i];
      const double bx = ref.x[i + 1];
      const double by = ref.y[i + 1];

      const double vx = bx - ax;
      const double vy = by - ay;
      const double seg_len_sq = vx * vx + vy * vy;

      double t = 0.0;
      if (seg_len_sq > 1e-12) {
        t = ((x0.x - ax) * vx + (x0.y - ay) * vy) / seg_len_sq;
        t = clampValue(t, 0.0, 1.0);
      }

      const double proj_x = ax + t * vx;
      const double proj_y = ay + t * vy;
      const double dx = x0.x - proj_x;
      const double dy = x0.y - proj_y;
      const double d2 = dx * dx + dy * dy;

      const double seg_len = std::sqrt(seg_len_sq);
      const double progress = distances[i] + t * seg_len;

      if (d2 + 1e-9 < best_d2 ||
          (std::abs(d2 - best_d2) <= 1e-9 && progress > best_progress)) {
        best_d2 = d2;
        best_progress = progress;
      }
    }

    return clampValue(best_progress, 0.0, distances.back());
  }

  void occupancyCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Ignoring occupancy grid in frame '%s'; expected '%s'",
          msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    occupancy_.origin_x = msg->info.origin.position.x;
    occupancy_.origin_y = msg->info.origin.position.y;
    occupancy_.resolution = msg->info.resolution;
    occupancy_.width = msg->info.width;
    occupancy_.height = msg->info.height;
    occupancy_.data = msg->data;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty path. Ignoring.");
      return;
    }

    nav_msgs::msg::Path path_in_control_frame = *msg;
    std::string source_frame = msg->header.frame_id;
    if (source_frame.empty() && !msg->poses.empty()) {
      source_frame = msg->poses.front().header.frame_id;
    }

    if (source_frame.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Ignoring path with empty frame_id; expected '%s'",
                  map_frame_.c_str());
      return;
    }

    if (source_frame != map_frame_) {
      path_in_control_frame.header.frame_id = map_frame_;
      path_in_control_frame.poses.clear();
      path_in_control_frame.poses.reserve(msg->poses.size());

      for (const auto &pose : msg->poses) {
        geometry_msgs::msg::PoseStamped input_pose = pose;
        if (input_pose.header.frame_id.empty()) {
          input_pose.header.frame_id = source_frame;
        }

        geometry_msgs::msg::PoseStamped output_pose;
        try {
          const auto transform = tf_buffer_.lookupTransform(
              map_frame_, input_pose.header.frame_id, input_pose.header.stamp,
              rclcpp::Duration::from_seconds(0.1));
          tf2::doTransform(input_pose, output_pose, transform);
        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(),
                      "Could not transform incoming path from '%s' to '%s': %s",
                      input_pose.header.frame_id.c_str(), map_frame_.c_str(),
                      ex.what());
          return;
        }

        output_pose.header.frame_id = map_frame_;
        path_in_control_frame.poses.push_back(output_pose);
      }

      RCLCPP_INFO(this->get_logger(),
                  "Transformed path from '%s' to '%s' before loading reference",
                  source_frame.c_str(), map_frame_.c_str());
    }

    TrajectoryReference ref = buildReferenceFromPath(path_in_control_frame);

    if (!ref.valid()) {
      RCLCPP_WARN(this->get_logger(), "Generated reference is invalid");
      return;
    }

    RobotState current_state;
    const bool should_resume_tracking =
        active_ && has_reference_ && !reference_.empty() &&
        lookupCurrentState(current_state);
    double carry_progress = 0.0;
    if (should_resume_tracking) {
      if (reference_s_.size() != reference_.size()) {
        reference_s_ = buildCumulativeDistances(reference_);
      }
      // Keep the already-traveled arc length when the same path is republished
      // with additional samples or extra clicked waypoints.
      carry_progress = estimateProgressAlongReference(
          current_state, reference_, reference_s_, step_);
    }

    std::vector<double> new_reference_s = buildCumulativeDistances(ref);
    reference_ = std::move(ref);
    reference_s_ = std::move(new_reference_s);
    active_ = true;
    has_reference_ = true;
    opt_states_cache_.clear();

    if (should_resume_tracking) {
      progress_s_ = clampValue(carry_progress, 0.0, reference_s_.back());
      step_ = stepFromProgress(reference_s_, progress_s_);
      const double refined_progress = estimateProgressAlongReference(
          current_state, reference_, reference_s_, step_);
      progress_s_ = std::max(progress_s_, refined_progress);
      step_ = stepFromProgress(reference_s_, progress_s_);
      RCLCPP_INFO(this->get_logger(),
                  "Reference updated with %zu samples; resuming at s=%.3f step=%zu",
                  reference_.size(), progress_s_, step_);
      return;
    }

    step_ = 0;
    progress_s_ = 0.0;
    current_vr_ = 0.0;
    current_vl_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Reference loaded with %zu samples",
                reference_.size());
  }

  TrajectoryReference
  buildReferenceFromPath(const nav_msgs::msg::Path &path_msg) const {
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
        ref.theta[i] = quaternionToYaw(path_msg.poses[i].pose.orientation);
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

    // The last sample is treated as a terminal point where the robot should
    // settle, so the desired speed and turn rate are set to zero.
    linear_v[n - 1] = 0.0;
    angular_v[n - 1] = 0.0;

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
      ref.ar[i] = clampValue((ref.vr[i] - ref.vr[i - 1]) / config_.h,
                             -config_.a_max, config_.a_max);
      ref.al[i] = clampValue((ref.vl[i] - ref.vl[i - 1]) / config_.h,
                             -config_.a_max, config_.a_max);
    }

    return ref;
  }

  bool lookupCurrentState(RobotState &x0) {
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_.lookupTransform(map_frame_, base_frame_,
                                          tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "TF lookup failed: %s", ex.what());
      return false;
    }

    x0.x = tf_msg.transform.translation.x;
    x0.y = tf_msg.transform.translation.y;
    x0.theta = quaternionToYaw(tf_msg.transform.rotation);
    x0.vr = current_vr_;
    x0.vl = current_vl_;

    return true;
  }

  // Publish a zero command and reset the internal wheel-speed estimate used as
  // the next state initialization.
  void publishStop() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);

    current_vr_ = 0.0;
    current_vl_ = 0.0;
  }

  void controlLoop() {
    if (!active_ || !has_reference_) {
      return;
    }

    if (reference_.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Reference is empty");
      publishStop();
      active_ = false;
      return;
    }

    RobotState x0;
    if (!lookupCurrentState(x0)) {
      RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "NMPC waiting for TF/current state: step=%zu path_samples=%zu",
          step_, reference_.size());
      publishStop();
      return;
    }

    if (reference_s_.size() != reference_.size()) {
      reference_s_ = buildCumulativeDistances(reference_);
    }

    // Progress is kept monotonic so the controller does not jump backwards when
    // the path contains dense or nearly overlapping samples.
    const double estimated_progress = estimateProgressAlongReference(
        x0, reference_, reference_s_, step_);
    progress_s_ = std::max(progress_s_, estimated_progress);
    step_ = stepFromProgress(reference_s_, progress_s_);

    const std::size_t last_idx = reference_.size() - 1;
    const double dx_goal = x0.x - reference_.x[last_idx];
    const double dy_goal = x0.y - reference_.y[last_idx];
    const double goal_distance = std::hypot(dx_goal, dy_goal);

    RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "NMPC status: step=%zu progress_s=%.3f goal_dist=%.3f ref_size=%zu costmap=%zux%zu",
        step_, progress_s_, goal_distance, reference_.size(),
        occupancy_.width, occupancy_.height);

    if (goal_distance < goal_tolerance_) {
      RCLCPP_INFO(this->get_logger(),
                  "Goal reached: goal_dist=%.3f tolerance=%.3f step=%zu ref_size=%zu",
                  goal_distance, goal_tolerance_, step_, reference_.size());
      publishStop();
      active_ = false;
      return;
    }

    const SolveResult result =
        solve(step_, reference_.size(), reference_, x0, occupancy_);

    if (!result.success) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "NMPC solve failed: %s", result.message.c_str());
      RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "NMPC solve context: step=%zu progress_s=%.3f goal_dist=%.3f obstacles=%zu data_time=%.6f solver_time=%.6f",
          step_, progress_s_, goal_distance, result.voxel_obstacles.size(),
          result.data_time, result.solver_time);
      publishStop();
      return;
    }

    if (result.vr_horizon.size() < 2 || result.vl_horizon.size() < 2) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "NMPC returned invalid control horizon");
      RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "NMPC invalid horizon sizes: vr=%zu vl=%zu step=%zu",
          result.vr_horizon.size(), result.vl_horizon.size(), step_);
      publishStop();
      return;
    }

    const double vr_cmd = result.vr_horizon.at(1);
    const double vl_cmd = result.vl_horizon.at(1);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = (vr_cmd + vl_cmd) / 2.0;
    cmd.angular.z = (vr_cmd - vl_cmd) / config_.L;
    cmd_vel_pub_->publish(cmd);

    current_vr_ = vr_cmd;
    current_vl_ = vl_cmd;

    RCLCPP_DEBUG(
        this->get_logger(),
        "step=%zu solver_time=%.6f data_time=%.6f obstacles=%zu v=%.4f w=%.4f",
        step_, result.solver_time, result.data_time,
        result.voxel_obstacles.size(), cmd.linear.x, cmd.angular.z);

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
  double goal_tolerance_{0.05};
  double progress_s_{0.0};
  double current_vr_{0.0};
  double current_vl_{0.0};

  std::vector<double> reference_s_{};

  bool active_{false};
  bool has_reference_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMPCControllerNode>());
  rclcpp::shutdown();
  return 0;
}
