#pragma once

#include <string>
#include <behaviortree_cpp/condition_node.h>

namespace rover_bt {

/**
 * @brief SUCCESS when the blackboard `command` equals the `expected` port value.
 *
 * Used to branch the tree on the currently-latched command. FAILURE on mismatch
 * or a missing `expected` port (a missing `command` is treated as empty).
 */
class CheckCommand : public BT::ConditionNode {
public:
  CheckCommand(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
