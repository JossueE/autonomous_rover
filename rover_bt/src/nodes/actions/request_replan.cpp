#include "rover_bt/nodes/actions/request_replan.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

RequestReplan::RequestReplan(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList RequestReplan::providedPorts() {
  return {};
}

BT::NodeStatus RequestReplan::tick() {
  std::shared_ptr<SharedContext> ctx;
  if (config().blackboard->get("context", ctx) && ctx && ctx->node) {
    RCLCPP_INFO(ctx->node->get_logger(), "RequestReplan: requested a new route/replan from the path planner");
    // Action goal resend or path planning request would go here in Phase 3.
  }

  // Transition nav_status back to navigating to resume navigation checks
  config().blackboard->set("nav_status", std::string("navigating"));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
