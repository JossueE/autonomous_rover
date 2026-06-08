#include "rover_bt/nodes/actions/zero_twist.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

ZeroTwist::ZeroTwist(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList ZeroTwist::providedPorts() {
  return {};
}

BT::NodeStatus ZeroTwist::tick() {
  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->cmd_vel_pub) {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::Twist twist;  // defaults to all zeros
  ctx->cmd_vel_pub->publish(twist);
  
  RCLCPP_DEBUG(ctx->node->get_logger(), "ZeroTwist: published stop command");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
