#pragma once

#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Clears the mapping state (`is_mapping`=false, `mapping_mode`="off").
 *
 * Counterpart to StartMapping; the mapping-node lifecycle teardown is a
 * future-phase integration. Always returns SUCCESS.
 */
class StopMapping : public BT::SyncActionNode {
public:
  StopMapping(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rover_bt
