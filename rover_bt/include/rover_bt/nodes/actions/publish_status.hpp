#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Logs, and speaks via TTS, a human-readable snapshot of the robot's
 *        current mode, last command, target and navigation status.
 *
 * On-demand status node (distinct from the node's periodic 1 Hz RoverStatus
 * publication); intended for an explicit "status" voice/GUI command. Always
 * returns SUCCESS.
 */
class PublishStatus : public BT::SyncActionNode {
public:
  PublishStatus(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
