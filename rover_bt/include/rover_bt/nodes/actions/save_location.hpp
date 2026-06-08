#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Persists the robot's current pose as a named waypoint via the
 *        LocationRegistry (which writes it through to waypoints.yaml).
 *
 * Reads the live pose from the SharedContext at tick time. FAILURE if the
 * context/registry is missing or the registry write fails; SUCCESS otherwise.
 */
class SaveLocation : public BT::SyncActionNode {
public:
  SaveLocation(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
