#include "rover_bt/nodes/actions/stop_mapping.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

StopMapping::StopMapping(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList StopMapping::providedPorts() {
  return {};
}

BT::NodeStatus StopMapping::tick() {
  config().blackboard->set("is_mapping", false);
  config().blackboard->set("mapping_mode", std::string("off"));

  std::shared_ptr<SharedContext> ctx;
  if (config().blackboard->get("context", ctx) && ctx && ctx->node) {
    RCLCPP_INFO(ctx->node->get_logger(), "StopMapping: stopped mapping node");
    // ROS2 Lifecycle client calls would go here in Phase 3.
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
