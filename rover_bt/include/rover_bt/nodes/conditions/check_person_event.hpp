#pragma once

#include <string>
#include <behaviortree_cpp/condition_node.h>

namespace rover_bt {

/**
 * @brief Consumes a one-shot person-tracker edge event set by on_person_status.
 *
 * Port `event` selects which edge to drain:
 *   - "lost"  → the follower reached LOST_STOPPED (gave up searching)
 *   - "found" → the follower re-acquired the person after losing it
 *
 * Returns SUCCESS exactly once per event (clearing the flag via an atomic
 * compare-exchange), so a guarded Speak fires a single time on the transition.
 * FAILURE otherwise, or on an unknown event name.
 */
class CheckPersonEvent : public BT::ConditionNode {
public:
  CheckPersonEvent(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
