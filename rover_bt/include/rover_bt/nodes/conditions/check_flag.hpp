#pragma once

#include <string>
#include <behaviortree_cpp/condition_node.h>

namespace rover_bt {

class CheckFlag : public BT::ConditionNode {
public:
  CheckFlag(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
