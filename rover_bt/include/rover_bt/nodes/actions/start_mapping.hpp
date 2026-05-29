#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

class StartMapping : public BT::SyncActionNode {
public:
  StartMapping(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
