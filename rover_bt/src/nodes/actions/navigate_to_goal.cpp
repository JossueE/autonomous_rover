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

  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->node) {
    return BT::NodeStatus::FAILURE;
  }

  if (!sendGoalForLocation(loc_opt.value(), ctx)) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

bool NavigateToGoal::sendGoalForLocation(const std::string& location,
                                         const std::shared_ptr<SharedContext>& ctx) {
  target_location_ = location;
  // Claim a fresh token for this dispatch; callbacks from any prior goal hold an
  // older one and will be ignored (see goal_sequence_ in the header).
  const uint64_t sequence = goal_sequence_.fetch_add(1) + 1;

  RCLCPP_INFO(ctx->node->get_logger(),
              "NavigateToGoal: Starting Action Client navigation to '%s'",
              target_location_.c_str());

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
      return false;
    }
  } else {
    RCLCPP_ERROR(ctx->node->get_logger(), "NavigateToGoal: LocationRegistry is null!");
    config().blackboard->set("nav_status", std::string("failed"));
    return false;
  }

  // Created lazily on first navigation and reused for the node's lifetime.
  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<ActionType>(ctx->node, "navigate_to_goal");
  }

  // Bounded wait: this runs inside a BT tick and must not block the tree. If the
  // server is down, report planner_timeout and let the supervisor's recovery
  // branch react instead of wedging navigation.
  if (!action_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
    RCLCPP_ERROR(ctx->node->get_logger(), "NavigateToGoal: Action server 'navigate_to_goal' not available!");
    config().blackboard->set("nav_status", std::string("planner_timeout"));
    return false;
  }

  auto goal_msg = ActionType::Goal();
  goal_msg.location_name = target_location_;
  goal_msg.lanelet_name = lanelet_name;
  goal_msg.target_pose.header.frame_id = "map";
  goal_msg.target_pose.header.stamp = ctx->node->now();
  goal_msg.target_pose.pose.position.x = x;
  goal_msg.target_pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  goal_msg.target_pose.pose.orientation.x = q.x();
  goal_msg.target_pose.pose.orientation.y = q.y();
  goal_msg.target_pose.pose.orientation.z = q.z();
  goal_msg.target_pose.pose.orientation.w = q.w();

  // Each callback below captures `sequence` by value and bails the moment the
  // token advances (goal preempted/retargeted/halted), so a late reply from a
  // superseded goal can't flip the flags or blackboard of the current one.
  auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();

  send_goal_options.goal_response_callback = [this, ctx, sequence](std::shared_ptr<GoalHandle> handle) {
    if (sequence != goal_sequence_.load()) {
      return;
    }
    if (!handle) {
      RCLCPP_ERROR(ctx->node->get_logger(), "NavigateToGoal: Goal request rejected by action server");
      goal_failed_.store(true);
    } else {
      RCLCPP_INFO(ctx->node->get_logger(), "NavigateToGoal: Goal request accepted by action server");
      goal_accepted_.store(true);
      goal_handle_ = handle;
    }
  };

  send_goal_options.feedback_callback = [this, ctx, sequence](std::shared_ptr<GoalHandle> handle, const std::shared_ptr<const ActionType::Feedback> feedback) {
    (void)handle;
    if (sequence != goal_sequence_.load()) {
      return;
    }
    RCLCPP_DEBUG(ctx->node->get_logger(), "NavigateToGoal Feedback: distance remaining: %.2f", feedback->distance_remaining);
    config().blackboard->set("goal_distance", static_cast<double>(feedback->distance_remaining));
  };

  send_goal_options.result_callback = [this, ctx, sequence](const GoalHandle::WrappedResult& result) {
    if (sequence != goal_sequence_.load()) {
      return;
    }
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

  return true;
}

BT::NodeStatus NavigateToGoal::onRunning() {
  // Hot-retarget: if the location port changes mid-flight, cancel the active
  // goal and dispatch the new one while staying RUNNING, so the tree sees one
  // continuous navigation action rather than a failure/restart.
  auto loc_opt = getInput<std::string>("location");
  if (loc_opt && loc_opt.value() != target_location_) {
    std::shared_ptr<SharedContext> ctx;
    if (!config().blackboard->get("context", ctx) || !ctx || !ctx->node) {
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(ctx->node->get_logger(),
                "NavigateToGoal: target changed from '%s' to '%s'. Preempting active goal.",
                target_location_.c_str(), loc_opt.value().c_str());
    cancelActiveGoal(ctx, "target changed");
    if (!sendGoalForLocation(loc_opt.value(), ctx)) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (goal_failed_.load()) {
    return BT::NodeStatus::FAILURE;
  }
  if (goal_completed_.load()) {
    return BT::NodeStatus::SUCCESS;
  }
  config().blackboard->set("nav_status", std::string("navigating"));
  return BT::NodeStatus::RUNNING;
}

void NavigateToGoal::cancelActiveGoal(const std::shared_ptr<SharedContext>& ctx,
                                      const std::string& reason) {
  if (action_client_ && goal_handle_ &&
      !goal_completed_.load() && !goal_failed_.load()) {
    RCLCPP_INFO(ctx->node->get_logger(),
                "NavigateToGoal: Canceling active goal (%s).", reason.c_str());
    try {
      action_client_->async_cancel_goal(goal_handle_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "NavigateToGoal: cancel skipped: %s", e.what());
    }
  }
}

void NavigateToGoal::onHalted() {
  std::shared_ptr<SharedContext> ctx;
  if (config().blackboard->get("context", ctx) && ctx && ctx->node) {
    // Only cancel a goal that is still active. After a goal reaches a terminal
    // state (succeeded/failed), the supervising ReactiveFallback halts this node
    // (e.g. GoalReached fires on the next tick) and onHalted runs — but the goal
    // handle is no longer known to the action client, so async_cancel_goal would
    // throw rclcpp_action's "Goal handle is not known to this client", crashing
    // the whole BT node. Skip cancellation for already-terminal goals, and guard
    // the live-cancel path defensively so a late state transition can't abort us.
    cancelActiveGoal(ctx, "halted");
  }
  goal_sequence_.fetch_add(1);
  config().blackboard->set("nav_status", std::string("idle"));
}

}  // namespace rover_bt
