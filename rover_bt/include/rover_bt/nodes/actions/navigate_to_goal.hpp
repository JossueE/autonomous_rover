#pragma once

#include <string>
#include <future>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include "rover_bt/action/navigate_to_goal.hpp"

namespace rover_bt {

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

  rclcpp_action::Client<ActionType>::SharedPtr action_client_;
  std::shared_ptr<GoalHandle> goal_handle_;

  bool goal_sent_ = false;
  std::atomic<bool> goal_accepted_{false};
  std::atomic<bool> goal_completed_{false};
  std::atomic<bool> goal_failed_{false};

  std::string target_location_;
};

}  // namespace rover_bt
