#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Requests a fresh route/replan and flips `nav_status` back to
 *        "navigating" so the supervisor resumes navigation checks.
 *
 * One rung of the tiered recovery ladder: invoked once per failed goal (gated by
 * `recovery_attempted`) before the tree gives up. The actual planner re-request
 * is a future-phase integration; today this only re-arms the nav status. Always
 * returns SUCCESS.
 */
class RequestReplan : public BT::SyncActionNode {
public:
  RequestReplan(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
