#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Sets a named boolean blackboard key, parsing the value from a string.
 *
 * Truthy strings are "true"/"1"/"TRUE"; anything else is false. Used by the tree
 * to raise/clear coordination flags (e.g. emergency latches). FAILURE only if
 * the `flag`/`value` ports are missing; SUCCESS otherwise.
 */
class SetFlag : public BT::SyncActionNode {
public:
  SetFlag(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
