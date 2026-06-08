#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Writes the robot's high-level `mode` to the blackboard.
 *
 * `mode` is the single source of truth that drives the tree's mode-dispatch
 * switch and the per-tick person-tracker enable. FAILURE if the `mode` port is
 * missing; SUCCESS otherwise.
 */
class SetMode : public BT::SyncActionNode {
public:
  SetMode(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
