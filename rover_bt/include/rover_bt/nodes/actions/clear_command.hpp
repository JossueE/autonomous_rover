#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Clears the transient `command` blackboard key once it has been
 *        dispatched, so it isn't re-processed on the next tick.
 *
 * Deliberately leaves `target_location` untouched: that is durable state
 * consumed later by the AUTONOMOUS/PATROL supervisor and status reporting.
 * Always returns SUCCESS.
 */
class ClearCommand : public BT::SyncActionNode {
public:
  ClearCommand(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
