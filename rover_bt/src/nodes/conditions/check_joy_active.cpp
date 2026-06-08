#include "rover_bt/nodes/conditions/check_joy_active.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

CheckJoyActive::CheckJoyActive(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList CheckJoyActive::providedPorts() {
  return {
    BT::InputPort<double>("timeout_sec", 1.0,
                          "Seconds since last joystick use before considered idle")
  };
}

BT::NodeStatus CheckJoyActive::tick() {
  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->node) {
    return BT::NodeStatus::FAILURE;
  }

  double timeout = 1.0;
  getInput("timeout_sec", timeout);

  double last = ctx->last_joy_active_time.load();
  if (last <= 0.0) {
    // Joystick never used since startup.
    return BT::NodeStatus::FAILURE;
  }

  double now = ctx->node->get_clock()->now().seconds();
  if (now - last <= timeout) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace rover_bt
