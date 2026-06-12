#pragma once

#include <string>
#include <behaviortree_cpp/action_node.h>

namespace rover_bt {

/**
 * @brief Open-loop timed velocity move: drives at a fixed linear/angular twist
 *        for a fixed duration, then stops.
 *
 * Stateful so onRunning can re-publish the twist every tick (keeping cmd_vel
 * fresh for safety watchdogs) and stop cleanly when the duration elapses or the
 * node is halted. Ports: linear (m/s), angular (rad/s), duration (s). RUNNING
 * until the duration elapses, then SUCCESS; FAILURE if ports/context are missing.
 */
class MoveRover : public BT::StatefulActionNode {
public:
  MoveRover(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  double linear_ = 0.0;
  double angular_ = 0.0;
  double duration_ = 0.0;
  double start_time_ = 0.0;
};

}  // namespace rover_bt
