#include "rover_bt/nodes/actions/increment_patrol_index.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

IncrementPatrolIndex::IncrementPatrolIndex(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList IncrementPatrolIndex::providedPorts() {
  return {};
}

BT::NodeStatus IncrementPatrolIndex::tick() {
  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx) {
    return BT::NodeStatus::FAILURE;
  }

  std::lock_guard<std::mutex> lock(ctx->waypoints_mutex);
  if (ctx->patrol_waypoints.empty()) {
    RCLCPP_WARN(ctx->node->get_logger(), "IncrementPatrolIndex: patrol waypoints list is empty!");
    return BT::NodeStatus::FAILURE;
  }

  int index = -1;
  (void)config().blackboard->get("patrol_index", index);

  index = (index + 1) % static_cast<int>(ctx->patrol_waypoints.size());
  std::string next_wp = ctx->patrol_waypoints[index];

  config().blackboard->set("patrol_index", index);
  config().blackboard->set("patrol_waypoint", next_wp);

  // Set target_location blackboard key too, so navigate_to_goal and Speak can use it
  config().blackboard->set("target_location", next_wp);

  // Clear any terminal nav_status so the just-advanced waypoint isn't seen as
  // already-reached on the next tick (which would skip waypoints).
  config().blackboard->set("nav_status", std::string("navigating"));

  RCLCPP_INFO(ctx->node->get_logger(), "IncrementPatrolIndex: advanced patrol index to %d, waypoint '%s'",
              index, next_wp.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
