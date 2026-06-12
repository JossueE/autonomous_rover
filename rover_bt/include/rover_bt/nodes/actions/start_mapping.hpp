#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Marks the blackboard as mapping (`is_mapping`=true, `mapping_mode`).
 *
 * Records intent for status/branching; the actual mapping-node lifecycle bring-up
 * is a future-phase integration. Always returns SUCCESS.
 */
class StartMapping : public BT::SyncActionNode {
public:
  StartMapping(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
