#include "rover_bt/nodes/conditions/check_nav_status.hpp"

namespace rover_bt {

CheckNavStatus::CheckNavStatus(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList CheckNavStatus::providedPorts() {
  return {
    BT::InputPort<std::string>("expected", "Expected navigation status: idle, navigating, succeeded, failed, stalled, planner_timeout")
  };
}

BT::NodeStatus CheckNavStatus::tick() {
  auto expected = getInput<std::string>("expected");
  if (!expected) {
    return BT::NodeStatus::FAILURE;
  }

  std::string actual;
  if (!config().blackboard->get("nav_status", actual)) {
    actual = "idle";
  }

  if (actual == expected.value()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace rover_bt
