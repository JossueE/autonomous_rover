#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

class SaveLocation : public BT::SyncActionNode {
public:
  SaveLocation(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
