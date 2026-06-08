#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

// Publish an indoor/outdoor tuning-profile switch to the person_tracker node
// (std_msgs/String on ctx->person_profile_pub). An empty/whitespace profile is a
// no-op (SUCCESS without publishing), so entering PERSON_TRACK without a target
// keeps the tracker's current profile.
class SetPersonProfile : public BT::SyncActionNode {
public:
  SetPersonProfile(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
