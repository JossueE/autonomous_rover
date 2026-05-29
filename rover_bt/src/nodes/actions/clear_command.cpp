#include "rover_bt/nodes/actions/clear_command.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

ClearCommand::ClearCommand(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList ClearCommand::providedPorts() {
  return {};
}

BT::NodeStatus ClearCommand::tick() {
  config().blackboard->set("command", std::string(""));
  config().blackboard->set("target_location", std::string(""));

  std::shared_ptr<SharedContext> ctx;
  if (config().blackboard->get("context", ctx) && ctx && ctx->node) {
    RCLCPP_DEBUG(ctx->node->get_logger(), "ClearCommand: cleared command and target from blackboard");
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
