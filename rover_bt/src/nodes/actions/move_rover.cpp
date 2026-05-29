#include "rover_bt/nodes/actions/move_rover.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

MoveRover::MoveRover(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::PortsList MoveRover::providedPorts() {
  return {
    BT::InputPort<double>("linear", 0.0, "Linear velocity (m/s)"),
    BT::InputPort<double>("angular", 0.0, "Angular velocity (rad/s)"),
    BT::InputPort<double>("duration", 1.0, "Motion duration in seconds")
  };
}

BT::NodeStatus MoveRover::onStart() {
  auto lin_opt = getInput<double>("linear");
  auto ang_opt = getInput<double>("angular");
  auto dur_opt = getInput<double>("duration");

  if (!lin_opt || !ang_opt || !dur_opt) {
    return BT::NodeStatus::FAILURE;
  }

  linear_ = lin_opt.value();
  angular_ = ang_opt.value();
  duration_ = dur_opt.value();

  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->node || !ctx->cmd_vel_pub) {
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = ctx->node->get_clock()->now().seconds();

  RCLCPP_INFO(ctx->node->get_logger(), "MoveRover: start open-loop move (lin: %.2f, ang: %.2f) for %.2f seconds",
              linear_, angular_, duration_);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_;
  twist.angular.z = angular_;
  ctx->cmd_vel_pub->publish(twist);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveRover::onRunning() {
  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->node || !ctx->cmd_vel_pub) {
    return BT::NodeStatus::FAILURE;
  }

  double now = ctx->node->get_clock()->now().seconds();
  if (now - start_time_ >= duration_) {
    // Done! Stop the rover
    geometry_msgs::msg::Twist twist;
    ctx->cmd_vel_pub->publish(twist);
    RCLCPP_INFO(ctx->node->get_logger(), "MoveRover: finished open-loop move");
    return BT::NodeStatus::SUCCESS;
  }

  // Keep publishing twist for safety/continuity
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_;
  twist.angular.z = angular_;
  ctx->cmd_vel_pub->publish(twist);

  return BT::NodeStatus::RUNNING;
}

void MoveRover::onHalted() {
  std::shared_ptr<SharedContext> ctx;
  if (config().blackboard->get("context", ctx) && ctx && ctx->cmd_vel_pub) {
    geometry_msgs::msg::Twist twist;
    ctx->cmd_vel_pub->publish(twist);
    RCLCPP_INFO(ctx->node->get_logger(), "MoveRover: pre-empted, stopped rover");
  }
}

}  // namespace rover_bt
