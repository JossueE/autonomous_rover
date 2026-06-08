#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Publishes a single zero Twist on cmd_vel to stop the rover immediately.
 *
 * The tree's primary stop primitive (used in IDLE and emergency branches).
 * FAILURE if the context/publisher is missing; SUCCESS otherwise. Note that a
 * one-shot stop relies on the tree re-ticking it to keep cmd_vel fresh.
 */
class ZeroTwist : public BT::SyncActionNode {
public:
  ZeroTwist(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
