#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Pops the highest-priority queued command from the CommandArbitrator
 *        and lowers it onto the blackboard for the rest of the tree to act on.
 *
 * Sets `command`, `active_command_source` and (when present) `target_location`.
 * For navigate/patrol commands it also resets per-command state once here —
 * `nav_status` to "idle" and `recovery_attempted` to false — which must NOT live
 * in NavigateToGoal::onStart (that re-runs every tick and would loop recovery).
 * Always returns SUCCESS, including when the queue is empty.
 */
class ProcessCommand : public BT::SyncActionNode {
public:
  ProcessCommand(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
