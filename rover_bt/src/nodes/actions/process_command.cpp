#include "rover_bt/nodes/actions/process_command.hpp"
#include "rover_bt/shared_context.hpp"
#include "rover_bt/command_arbitrator.hpp"

namespace rover_bt {

ProcessCommand::ProcessCommand(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList ProcessCommand::providedPorts() {
  return {};
}

BT::NodeStatus ProcessCommand::tick() {
  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->arbitrator) {
    return BT::NodeStatus::FAILURE;
  }

  auto cmd_opt = ctx->arbitrator->consume();
  if (cmd_opt) {
    const auto& cmd = cmd_opt.value();
    config().blackboard->set("command", cmd.command);
    if (!cmd.target.empty()) {
      config().blackboard->set("target_location", cmd.target);
    }
    
    RCLCPP_INFO(ctx->node->get_logger(), "ProcessCommand: Consumed command [%s], target [%s], priority [%d] from source [%s]",
                cmd.command.c_str(), cmd.target.c_str(), cmd.priority, cmd.source.c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
