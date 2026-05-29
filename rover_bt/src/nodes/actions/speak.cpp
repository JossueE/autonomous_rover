#include "rover_bt/nodes/actions/speak.hpp"
#include "rover_bt/shared_context.hpp"
#include "rover_bt/tts_client.hpp"

namespace rover_bt {

Speak::Speak(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList Speak::providedPorts() {
  return {
    BT::InputPort<std::string>("message", "Message text to speak (supports {target_location} and {patrol_waypoint})")
  };
}

BT::NodeStatus Speak::tick() {
  auto message_opt = getInput<std::string>("message");
  if (!message_opt) {
    return BT::NodeStatus::FAILURE;
  }

  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx) {
    return BT::NodeStatus::FAILURE;
  }

  std::string text = message_opt.value();

  // Template substitutions
  std::string target_location = "";
  (void)config().blackboard->get("target_location", target_location);

  std::string patrol_waypoint = "";
  (void)config().blackboard->get("patrol_waypoint", patrol_waypoint);

  size_t pos;
  while ((pos = text.find("{target_location}")) != std::string::npos) {
    text.replace(pos, 17, target_location);
  }
  while ((pos = text.find("{patrol_waypoint}")) != std::string::npos) {
    text.replace(pos, 17, patrol_waypoint);
  }

  if (ctx->tts) {
    ctx->tts->speak(text);
  } else {
    RCLCPP_INFO(ctx->node->get_logger(), "[Speak Node - No TTS]: %s", text.c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
