#include "rover_bt/nodes/conditions/check_sensor_health.hpp"
#include "rover_bt/shared_context.hpp"

#include <cmath>
#include <string>

namespace rover_bt {

CheckSensorHealth::CheckSensorHealth(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList CheckSensorHealth::providedPorts() {
  return {
    BT::InputPort<std::string>("sensor", "Sensor name to check: odom, lidar, camera, imu, motor"),
    BT::InputPort<double>("max_stale_sec", 3.0, "Maximum stale time in seconds before failure"),
    BT::InputPort<std::string>("on_fail", "warn", "Action on failure: warn or error"),
    BT::InputPort<double>("startup_grace_sec", 15.0,
        "Seconds after node start during which a never-seen sensor is tolerated. "
        "After this, a sensor that never produced data is treated as stale."),
    BT::InputPort<bool>("require_moving", false,
        "If true, only escalate a stale sensor to FAILURE while the robot is "
        "actually moving (|linear_vel| > moving_speed). Used for the wheel gate: "
        "wheels going silent is only an emergency if we are trying to move."),
    BT::InputPort<double>("moving_speed", 0.05,
        "Speed threshold (m/s) above which the robot counts as moving, for require_moving.")
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
    // Lidar is sim-only on this robot. When monitoring is disabled (real HW)
    // the watchdog is a no-op so it never false-alarms on a sensor that does
    // not exist.
    if (!ctx->monitor_lidar.load()) {
      return BT::NodeStatus::SUCCESS;
    }
    last = ctx->last_lidar_time.load();
  } else if (name == "camera") {
    last = ctx->last_camera_time.load();
  } else if (name == "odom") {
    if (ctx->rtabmap_odom_lost.load()) {
      RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
                           "Sensor watchdog: RTAB-Map reports odometry lost");
      return BT::NodeStatus::FAILURE;
    }
    last = ctx->last_rtabmap_odom_info_time.load();
    if (last == 0.0) {
      last = ctx->node_start_time.load();
    }
  } else if (name == "imu") {
    last = ctx->last_imu_time.load();
  } else if (name == "motor") {
    // Wheel-encoder liveness. Disabled in sim (no wheel drivers there).
    if (!ctx->monitor_wheels.load()) {
      return BT::NodeStatus::SUCCESS;
    }
    last = ctx->last_motor_time.load();
  } else if (name == "trajectory") {
    last = ctx->last_trajectory_time.load();
  } else {
    RCLCPP_WARN(ctx->node->get_logger(), "CheckSensorHealth: unknown sensor '%s'", name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  double staleness;
  if (last == 0.0) {
    // Never received data. Tolerate this only during the bounded startup grace
    // window; after that, "never seen" is a genuine fault (e.g. the driver
    // never came up) and must be reported, not silently treated as healthy.
    double grace = getInput<double>("startup_grace_sec").value_or(15.0);
    double start = ctx->node_start_time.load();
    if (start == 0.0 || (now - start) < grace) {
      return BT::NodeStatus::SUCCESS;
    }
    staleness = now - start;  // treat as stale for the whole post-grace window
  } else {
    staleness = now - last;
    if (staleness <= max_stale.value()) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  // Stale. Warn-only watchdogs (and the SystemHealthMonitor) just log so the
  // tree keeps running; the real status is surfaced in /rover_bt/status.
  auto on_fail_opt = getInput<std::string>("on_fail");
  const bool warn_only = !on_fail_opt || on_fail_opt.value() == "warn";

  // require_moving: a stale sensor only becomes an emergency while the robot is
  // actually moving. Used for the wheel gate — silent wheels at idle are not
  // dangerous, but silent wheels while rolling are. robot_linear_vel comes from
  // /rtabmap/odom (visual odom), so it's an independent witness of motion even
  // when the wheel encoders themselves have gone quiet.
  if (!warn_only && getInput<bool>("require_moving").value_or(false)) {
    double speed = std::abs(ctx->robot_linear_vel.load());
    double moving_speed = getInput<double>("moving_speed").value_or(0.05);
    if (speed <= moving_speed) {
      // Stationary: log but do not e-stop.
      RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 5000,
                           "Sensor watchdog: '%s' stale by %.2fs while stationary "
                           "(no e-stop)", name.c_str(), staleness);
      return BT::NodeStatus::SUCCESS;
    }
  }

  RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
                       "Sensor watchdog: '%s' is stale by %.2f seconds (limit %.2f)",
                       name.c_str(), staleness, max_stale.value());

  return warn_only ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rover_bt
