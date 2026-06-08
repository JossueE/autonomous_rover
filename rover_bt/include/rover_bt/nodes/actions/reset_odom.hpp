#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

namespace rover_bt {

/**
 * @brief One-shot active odometry-recovery routine, run while the rover is held
 *        in an odom-loss e-stop (blackboard flag emergency_from_odom).
 *
 * RTAB-Map's visual/ICP odometry can stay "lost" indefinitely, so passively
 * waiting is not enough. delay_sec after the loss began, this node calls
 * /rtabmap/reset_odom ONCE, forcing the odometry layer to drop its lost track
 * and re-initialise from the current frame.
 *
 * Crucially, reset_odom only restores ODOMETRY — it does not itself relocalize.
 * Relocalization is the separate rtabmap (localization-mode) node's job: once
 * odometry flows again, rtabmap re-matches the live frames against the prior map
 * and re-pins the robot via loop closure, publishing /rtabmap/localization_pose.
 * So this routine judges success on a *relocalization* (a localization_pose
 * newer than the loss), NOT merely on odometry tracking resuming — "I know where
 * I am again", not just "odometry is running".
 *
 * Single attempt per loss episode (episode = one continuous emergency_from_odom):
 *   - attempt:  voice "Intentando recuperar la odometría." when the reset fires.
 *   - success:  a fresh localization_pose arrives → set blackboard
 *               odom_relocalized=true; the tree's AutoResumeFromOdom branch then
 *               returns to IDLE with its own confirmation.
 *   - failure:  still not relocalized verdict_sec after the attempt → voice the
 *               failure once; the e-stop is held. No further automatic attempts
 *               this episode, but a late relocalization still resumes (the
 *               success check keeps running).
 *
 * The BT ticks from a single-threaded executor, so the service call is async
 * (fire-and-forget) — the node never spins to wait. It ALWAYS returns SUCCESS:
 * it is a passive helper inside the emergency-hold sequence and must never make
 * that sequence fail. Episode state is latched on the rising edge of
 * emergency_from_odom, so it survives odometry recovering before relocalization.
 */
class ResetOdom : public BT::SyncActionNode {
public:
  ResetOdom(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  void speak(const std::string& text);

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;

  // Episode state. in_episode_ rising edge (emergency_from_odom false→true)
  // re-arms everything; recovery_ref_ is the node-clock time the episode began,
  // used to recognise a localization_pose that is newer than the loss.
  bool in_episode_{false};
  double recovery_ref_{0.0};
  bool attempted_{false};       // reset_odom already fired for this episode
  bool verdict_done_{false};    // failure already announced for this episode
  double attempt_time_{0.0};    // node-clock time the reset was sent
};

}  // namespace rover_bt
