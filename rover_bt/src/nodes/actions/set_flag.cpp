#include "rover_bt/nodes/actions/set_flag.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

SetFlag::SetFlag(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList SetFlag::providedPorts() {
  return {
    BT::InputPort<std::string>("flag", "Blackboard boolean flag key to set"),
    BT::InputPort<std::string>("value", "value: true or false")
  };
}

BT::NodeStatus SetFlag::tick() {
  auto flag_opt = getInput<std::string>("flag");
  auto val_opt = getInput<std::string>("value");

  if (!flag_opt || !val_opt) {
    return BT::NodeStatus::FAILURE;
  }

  std::string flag_name = flag_opt.value();
  std::string val_str = val_opt.value();
  bool val = (val_str == "true" || val_str == "1" || val_str == "TRUE");

  config().blackboard->set(flag_name, val);

  std::shared_ptr<SharedContext> ctx;
  if (config().blackboard->get("context", ctx) && ctx && ctx->node) {
    RCLCPP_DEBUG(ctx->node->get_logger(), "SetFlag: flag '%s' set to %s", flag_name.c_str(), val ? "true" : "false");
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
