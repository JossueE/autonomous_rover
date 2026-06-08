#include "rover_bt/nodes/conditions/check_mode.hpp"

namespace rover_bt {

CheckMode::CheckMode(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList CheckMode::providedPorts() {
  return {
    BT::InputPort<std::string>("expected", "The mode expected to match")
  };
}

BT::NodeStatus CheckMode::tick() {
  auto expected = getInput<std::string>("expected");
  if (!expected) {
    return BT::NodeStatus::FAILURE;
  }

  std::string actual;
  if (!config().blackboard->get("mode", actual)) {
    actual = "IDLE";  // default mode
  }

  if (actual == expected.value()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace rover_bt
