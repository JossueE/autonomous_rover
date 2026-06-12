#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Speaks a message via TTS, expanding {target_location} and
 *        {patrol_waypoint} placeholders from the blackboard.
 *
 * Falls back to logging when no TTS client is configured. FAILURE if the
 * `message` port or context is missing; SUCCESS otherwise.
 */
class Speak : public BT::SyncActionNode {
public:
  Speak(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
