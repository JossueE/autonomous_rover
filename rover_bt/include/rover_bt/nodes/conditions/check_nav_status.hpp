#pragma once

#include <string>
#include <behaviortree_cpp/condition_node.h>

namespace rover_bt {

/**
 * @brief SUCCESS when the blackboard `nav_status` equals the `expected` value.
 *
 * Lets the navigation supervisor branch on outcomes (idle, navigating,
 * succeeded, failed, stalled, planner_timeout). A missing status defaults to
 * "idle"; FAILURE on mismatch or a missing `expected` port.
 */
class CheckNavStatus : public BT::ConditionNode {
public:
  CheckNavStatus(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
