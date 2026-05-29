#include "rover_bt/nodes/actions/set_mode.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

SetMode::SetMode(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList SetMode::providedPorts() {
  return {
    BT::InputPort<std::string>("mode", "The new robot mode to set")
  };
}

BT::NodeStatus SetMode::tick() {
  auto mode_opt = getInput<std::string>("mode");
  if (!mode_opt) {
    return BT::NodeStatus::FAILURE;
  }

  std::string new_mode = mode_opt.value();
  config().blackboard->set("mode", new_mode);

  std::shared_ptr<SharedContext> ctx;
  if (config().blackboard->get("context", ctx) && ctx && ctx->node) {
    RCLCPP_INFO(ctx->node->get_logger(), "SetMode: Mode changed to '%s'", new_mode.c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
