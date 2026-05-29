#include "rover_bt/nodes/conditions/check_command.hpp"

namespace rover_bt {

CheckCommand::CheckCommand(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList CheckCommand::providedPorts() {
  return {
    BT::InputPort<std::string>("expected", "The command expected to match")
  };
}

BT::NodeStatus CheckCommand::tick() {
  auto expected = getInput<std::string>("expected");
  if (!expected) {
    return BT::NodeStatus::FAILURE;
  }

  std::string actual;
  if (!config().blackboard->get("command", actual)) {
    actual = "";
  }

  if (actual == expected.value()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace rover_bt
