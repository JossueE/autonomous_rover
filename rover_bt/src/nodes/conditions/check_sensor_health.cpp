#include "rover_bt/nodes/conditions/check_sensor_health.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

CheckSensorHealth::CheckSensorHealth(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList CheckSensorHealth::providedPorts() {
  return {
    BT::InputPort<std::string>("sensor", "Sensor name to check: odom, lidar, camera, imu, motor"),
    BT::InputPort<double>("max_stale_sec", 3.0, "Maximum stale time in seconds before failure"),
    BT::InputPort<std::string>("on_fail", "warn", "Action on failure: warn or error")
  };
}

BT::NodeStatus CheckSensorHealth::tick() {
  auto sensor = getInput<std::string>("sensor");
  auto max_stale = getInput<double>("max_stale_sec");
  if (!sensor || !max_stale) {
    return BT::NodeStatus::FAILURE;
  }

  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->node) {
    return BT::NodeStatus::FAILURE;
  }

  double now = ctx->node->get_clock()->now().seconds();
  double last = 0.0;

  const std::string& name = sensor.value();
  if (name == "lidar") {
    last = ctx->last_lidar_time.load();
  } else if (name == "camera") {
    last = ctx->last_camera_time.load();
  } else if (name == "odom") {
    last = ctx->last_odom_time.load();
  } else if (name == "imu") {
    last = ctx->last_imu_time.load();
  } else if (name == "motor") {
    last = ctx->last_motor_time.load();
  } else if (name == "trajectory") {
    last = ctx->last_trajectory_time.load();
  } else {
    RCLCPP_WARN(ctx->node->get_logger(), "CheckSensorHealth: unknown sensor '%s'", name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (last == 0.0) {
    // Startup grace period
    return BT::NodeStatus::SUCCESS;
  }

  double staleness = now - last;
  if (staleness <= max_stale.value()) {
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(ctx->node->get_logger(), "Sensor watchdog: '%s' is stale by %.2f seconds (limit %.2f)",
              name.c_str(), staleness, max_stale.value());

  auto on_fail_opt = getInput<std::string>("on_fail");
  if (on_fail_opt && on_fail_opt.value() == "warn") {
    // Return SUCCESS to not break the tree, but log warning
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rover_bt
