#pragma once

#include <string>
#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

class MoveRover : public BT::StatefulActionNode {
public:
  MoveRover(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  double linear_ = 0.0;
  double angular_ = 0.0;
  double duration_ = 0.0;
  double start_time_ = 0.0;
};

}  // namespace rover_bt
