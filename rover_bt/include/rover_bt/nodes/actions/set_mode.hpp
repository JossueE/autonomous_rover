#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

class SetMode : public BT::SyncActionNode {
public:
  SetMode(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
