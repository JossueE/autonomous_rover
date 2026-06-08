#pragma once

#include <string>
#include <behaviortree_cpp/condition_node.h>

namespace rover_bt {

// Consume a one-shot person-tracker edge event set by on_person_status:
//   event="lost"  → the follower reached LOST_STOPPED (gave up searching)
//   event="found" → the follower re-acquired the person after losing it
// Returns SUCCESS exactly once per event (clears the flag), so a guarded Speak
// fires a single time on the transition. FAILURE otherwise.
class CheckPersonEvent : public BT::ConditionNode {
public:
  CheckPersonEvent(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
