#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Advances the patrol cursor to the next waypoint (wrapping at the end)
 *        and arms it as the next navigation target.
 *
 * Writes the new index/name to `patrol_index`, `patrol_waypoint` and
 * `target_location`, and resets `nav_status` to "navigating" so the supervisor
 * starts a fresh goal instead of treating the new waypoint as already reached.
 * FAILURE if the patrol waypoint list is empty; SUCCESS otherwise.
 */
class IncrementPatrolIndex : public BT::SyncActionNode {
public:
  IncrementPatrolIndex(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
