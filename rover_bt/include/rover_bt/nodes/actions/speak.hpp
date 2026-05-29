#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

class Speak : public BT::SyncActionNode {
public:
  Speak(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
