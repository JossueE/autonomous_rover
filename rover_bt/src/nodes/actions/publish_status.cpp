#include "rover_bt/nodes/actions/publish_status.hpp"
#include "rover_bt/shared_context.hpp"
#include "rover_bt/tts_client.hpp"

namespace rover_bt {

PublishStatus::PublishStatus(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList PublishStatus::providedPorts() {
  return {};
}

BT::NodeStatus PublishStatus::tick() {
  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->node) {
    return BT::NodeStatus::FAILURE;
  }

  std::string mode = "IDLE";
  (void)config().blackboard->get("mode", mode);

  std::string last_command = "";
  (void)config().blackboard->get("command", last_command);

  std::string target_location = "";
  (void)config().blackboard->get("target_location", target_location);

  std::string nav_status = "idle";
  (void)config().blackboard->get("nav_status", nav_status);

  RCLCPP_INFO(ctx->node->get_logger(), "--- Robot Status ---");
  RCLCPP_INFO(ctx->node->get_logger(), "Mode: %s", mode.c_str());
  RCLCPP_INFO(ctx->node->get_logger(), "Last Command: %s", last_command.c_str());
  RCLCPP_INFO(ctx->node->get_logger(), "Target Location: %s", target_location.c_str());
  RCLCPP_INFO(ctx->node->get_logger(), "Nav Status: %s", nav_status.c_str());
  RCLCPP_INFO(ctx->node->get_logger(), "Pose: (x: %.2f, y: %.2f, theta: %.2f)",
              ctx->robot_x.load(), ctx->robot_y.load(), ctx->robot_theta.load());
  RCLCPP_INFO(ctx->node->get_logger(), "--------------------");

  if (ctx->tts) {
    std::string summary = "Estado del sistema. Modo: " + mode + ". ";
    if (mode == "AUTONOMOUS" || mode == "PATROL") {
      summary += "Navegando a: " + (mode == "AUTONOMOUS" ? target_location : target_location) + ". ";
    } else {
      summary += "Robot inactivo. ";
    }
    ctx->tts->speak(summary);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
