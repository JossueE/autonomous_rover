#pragma once

#include <string>
#include <behaviortree_cpp/condition_node.h>

namespace rover_bt {

/**
 * @brief SUCCESS when the blackboard `mode` equals the `expected` port value.
 *
 * The mode-dispatch guard for the tree's top-level behaviour branches. A missing
 * `mode` defaults to IDLE; FAILURE on mismatch or a missing `expected` port.
 */
class CheckMode : public BT::ConditionNode {
public:
  CheckMode(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
