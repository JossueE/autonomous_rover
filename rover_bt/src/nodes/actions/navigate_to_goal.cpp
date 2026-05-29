#include "rover_bt/nodes/actions/navigate_to_goal.hpp"
#include "rover_bt/shared_context.hpp"
#include "rover_bt/location_registry.hpp"
#include "rover_bt/tts_client.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace rover_bt {

NavigateToGoal::NavigateToGoal(const std::string& name, const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config) {}

BT::PortsList NavigateToGoal::providedPorts() {
  return {
    BT::InputPort<std::string>("location", "Name of the location to navigate to")
  };
}

BT::NodeStatus NavigateToGoal::onStart() {
  auto loc_opt = getInput<std::string>("location");
  if (!loc_opt) {
    return BT::NodeStatus::FAILURE;
  }
  target_location_ = loc_opt.value();

  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->node) {
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ctx->node->get_logger(), "NavigateToGoal: Starting Action Client navigation to '%s'", target_location_.c_str());

  // Reset state flags
  goal_sent_ = false;
  goal_accepted_.store(false);
  goal_completed_.store(false);
  goal_failed_.store(false);
  goal_handle_.reset();

  // NOTE: recovery_attempted is intentionally NOT reset here. This node is the
  // RUNNING default leaf of the AUTONOMOUS/PATROL supervisor and onStart runs
  // again every time navigation resumes (e.g. after RequestReplan), so resetting
  // here would loop the tiered recovery forever (replan, replan, ...). The flag
  // is reset once per fresh navigate/patrol command in ProcessCommand instead.

  // Find waypoint
  double x = 0.0, y = 0.0, theta = 0.0;
  std::string lanelet_name = "";
  if (ctx->location_registry) {
    auto loc = ctx->location_registry->find(target_location_);
    if (loc) {
      x = loc->x;
      y = loc->y;
      theta = loc->theta;
      lanelet_name = loc->lanelet_name;
      RCLCPP_INFO(ctx->node->get_logger(), "NavigateToGoal: Found waypoint '%s' at (%.2f, %.2f, %.2f)",
                  target_location_.c_str(), x, y, theta);
    } else {
      RCLCPP_WARN(ctx->node->get_logger(), "NavigateToGoal: Waypoint '%s' not registered! Action failed.",
                  target_location_.c_str());
      // Mark failed so the supervisor's recovery branch handles it gracefully
      // (TTS + back to IDLE) instead of a NaN goal or a wedged AUTONOMOUS mode.
      config().blackboard->set("nav_status", std::string("failed"));
      if (ctx->tts) {
        ctx->tts->speak("No conozco la ubicación " + target_location_);
      }
      return BT::NodeStatus::FAILURE;
    }
  } else {
    RCLCPP_ERROR(ctx->node->get_logger(), "NavigateToGoal: LocationRegistry is null!");
    config().blackboard->set("nav_status", std::string("failed"));
    return BT::NodeStatus::FAILURE;
  }

  // Create action client
  action_client_ = rclcpp_action::create_client<ActionType>(ctx->node, "navigate_to_goal");

  // Wait for Action Server to be available (briefly)
  if (!action_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
    RCLCPP_ERROR(ctx->node->get_logger(), "NavigateToGoal: Action server 'navigate_to_goal' not available!");
    config().blackboard->set("nav_status", std::string("planner_timeout"));
    return BT::NodeStatus::FAILURE;
  }

  // Setup goal message
  auto goal_msg = ActionType::Goal();
  goal_msg.location_name = target_location_;
  goal_msg.lanelet_name = lanelet_name;
  goal_msg.target_pose.header.frame_id = "map";
  goal_msg.target_pose.header.stamp = ctx->node->now();
  goal_msg.target_pose.pose.position.x = x;
  goal_msg.target_pose.pose.position.y = y;
  
  // Convert yaw to quaternion
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  goal_msg.target_pose.pose.orientation.x = q.x();
  goal_msg.target_pose.pose.orientation.y = q.y();
  goal_msg.target_pose.pose.orientation.z = q.z();
  goal_msg.target_pose.pose.orientation.w = q.w();

  // Send Options
  auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
  
  send_goal_options.goal_response_callback = [this, ctx](std::shared_ptr<GoalHandle> handle) {
    if (!handle) {
      RCLCPP_ERROR(ctx->node->get_logger(), "NavigateToGoal: Goal request rejected by action server");
      goal_failed_.store(true);
    } else {
      RCLCPP_INFO(ctx->node->get_logger(), "NavigateToGoal: Goal request accepted by action server");
      goal_accepted_.store(true);
      goal_handle_ = handle;
    }
  };

  send_goal_options.feedback_callback = [this, ctx](std::shared_ptr<GoalHandle> handle, const std::shared_ptr<const ActionType::Feedback> feedback) {
    (void)handle;
    RCLCPP_DEBUG(ctx->node->get_logger(), "NavigateToGoal Feedback: distance remaining: %.2f", feedback->distance_remaining);
    config().blackboard->set("goal_distance", static_cast<double>(feedback->distance_remaining));
  };

  send_goal_options.result_callback = [this, ctx](const GoalHandle::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->success) {
      RCLCPP_INFO(ctx->node->get_logger(), "NavigateToGoal: Arrived at goal successfully!");
      config().blackboard->set("nav_status", std::string("succeeded"));
      goal_completed_.store(true);
    } else {
      RCLCPP_ERROR(ctx->node->get_logger(), "NavigateToGoal: Goal execution failed or aborted (code %d)", static_cast<int>(result.code));
      config().blackboard->set("nav_status", std::string("failed"));
      goal_failed_.store(true);
    }
  };

  action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_sent_ = true;
  config().blackboard->set("nav_status", std::string("navigating"));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToGoal::onRunning() {
  if (goal_failed_.load()) {
    return BT::NodeStatus::FAILURE;
  }
  if (goal_completed_.load()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void NavigateToGoal::onHalted() {
  std::shared_ptr<SharedContext> ctx;
  if (config().blackboard->get("context", ctx) && ctx && ctx->node) {
    RCLCPP_INFO(ctx->node->get_logger(), "NavigateToGoal: Navigation halted. Canceling action goal.");
    if (action_client_ && goal_handle_) {
      action_client_->async_cancel_goal(goal_handle_);
    }
  }
  config().blackboard->set("nav_status", std::string("idle"));
}

}  // namespace rover_bt
