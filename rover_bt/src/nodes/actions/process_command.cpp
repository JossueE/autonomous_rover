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
    config().blackboard->set("active_command_source", cmd.source);
    if (!cmd.target.empty()) {
      config().blackboard->set("target_location", cmd.target);
    }

    // A fresh navigation request must clear any stale terminal nav_status
    // (e.g. "succeeded"/"failed" from a previous goal) so the AUTONOMOUS/PATROL
    // supervisor starts a new goal instead of immediately firing GoalReached.
    // It also starts a new recovery cycle (replan-once-then-give-up), so the
    // recovery_attempted flag is reset here — once per command — rather than in
    // NavigateToGoal::onStart, which re-runs every tick and would loop recovery.
    if (cmd.command == "navigate" || cmd.command == "patrol") {
      // Reset to "idle" (not "navigating"): NavigateToGoal::onStart sets
      // "navigating" once the goal is actually sent in AUTONOMOUS/PATROL. If the
      // command is discarded (e.g. arriving during EMERGENCY), nav_status then
      // correctly stays "idle" instead of falsely reporting "navigating".
      config().blackboard->set("nav_status", std::string("idle"));
      config().blackboard->set("recovery_attempted", false);
    }
    
    RCLCPP_INFO(ctx->node->get_logger(), "ProcessCommand: Consumed command [%s], target [%s], priority [%d] from source [%s]",
                cmd.command.c_str(), cmd.target.c_str(), cmd.priority, cmd.source.c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
