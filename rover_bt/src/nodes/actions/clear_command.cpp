#include "rover_bt/nodes/actions/clear_command.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

ClearCommand::ClearCommand(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList ClearCommand::providedPorts() {
  return {};
}

BT::NodeStatus ClearCommand::tick() {
  // Only clear the transient command. target_location is durable state (the
  // current/last navigation target) consumed later by the AUTONOMOUS/PATROL
  // supervisor and by status reporting, so it must NOT be wiped here.
  config().blackboard->set("command", std::string(""));

  std::shared_ptr<SharedContext> ctx;
  if (config().blackboard->get("context", ctx) && ctx && ctx->node) {
    RCLCPP_DEBUG(ctx->node->get_logger(), "ClearCommand: cleared command from blackboard");
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
