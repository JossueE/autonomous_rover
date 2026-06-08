#pragma once

#include <atomic>
#include <string>
#include <future>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include "rover_bt/action/navigate_to_goal.hpp"

namespace rover_bt {

struct SharedContext;

class NavigateToGoal : public BT::StatefulActionNode {
public:
  NavigateToGoal(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using ActionType = rover_bt::action::NavigateToGoal;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

  bool sendGoalForLocation(const std::string& location,
                           const std::shared_ptr<SharedContext>& ctx);
  void cancelActiveGoal(const std::shared_ptr<SharedContext>& ctx,
                        const std::string& reason);

  rclcpp_action::Client<ActionType>::SharedPtr action_client_;
  std::shared_ptr<GoalHandle> goal_handle_;

  bool goal_sent_ = false;
  std::atomic<bool> goal_accepted_{false};
  std::atomic<bool> goal_completed_{false};
  std::atomic<bool> goal_failed_{false};
  std::atomic<uint64_t> goal_sequence_{0};

  std::string target_location_;
};

}  // namespace rover_bt
