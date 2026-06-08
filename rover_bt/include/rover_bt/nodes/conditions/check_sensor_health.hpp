#pragma once

#include <string>
#include <behaviortree_cpp/condition_node.h>

namespace rover_bt {

/**
 * @brief Watchdog that checks one sensor's liveness against a staleness budget.
 *
 * Compares now against the sensor's last-seen timestamp in the SharedContext for
 * the named sensor (odom, lidar, camera, imu, motor, trajectory). Behaviour is
 * tuned by ports: `on_fail` ("warn" logs and stays SUCCESS so the tree keeps
 * running; "error" returns FAILURE to trip an e-stop branch), a bounded
 * `startup_grace_sec` for never-seen sensors, optional `require_moving` gating,
 * and odom-specific `lost_debounce_sec` for RTAB-Map tracking dropouts. Disabled
 * watchdogs (lidar on real HW, wheels in sim) short-circuit to SUCCESS.
 */
class CheckSensorHealth : public BT::ConditionNode {
public:
  CheckSensorHealth(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
