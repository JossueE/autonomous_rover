#pragma once

#include <string>
#include <behaviortree_cpp/condition_node.h>

namespace rover_bt {

/**
 * @brief SUCCESS when the named boolean blackboard `flag` is set; supports
 *        inversion via the `negate` port.
 *
 * A missing flag reads as false. FAILURE if the `flag` port itself is absent.
 */
class CheckFlag : public BT::ConditionNode {
public:
  CheckFlag(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
