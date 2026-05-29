#include "rover_bt/nodes/conditions/check_flag.hpp"

namespace rover_bt {

CheckFlag::CheckFlag(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList CheckFlag::providedPorts() {
  return {
    BT::InputPort<std::string>("flag", "Blackboard boolean flag key to check"),
    BT::InputPort<bool>("negate", false, "If true, invert the checked boolean value")
  };
}

BT::NodeStatus CheckFlag::tick() {
  auto flag_opt = getInput<std::string>("flag");
  if (!flag_opt) {
    return BT::NodeStatus::FAILURE;
  }

  bool actual = false;
  (void)config().blackboard->get(flag_opt.value(), actual);

  bool negate = false;
  getInput("negate", negate);

  bool result = negate ? !actual : actual;
  return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rover_bt
