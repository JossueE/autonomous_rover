#pragma once

#include <string>
#include <behaviortree_cpp/condition_node.h>

namespace rover_bt {

// SUCCESS while the joystick has been used (deadman held) within the last
// `timeout_sec` seconds; FAILURE once it goes idle. Drives automatic entry to
// and exit from TELEOP_JOYCON without any explicit voice command.
class CheckJoyActive : public BT::ConditionNode {
public:
  CheckJoyActive(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
