#include "rover_bt/nodes/actions/start_mapping.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

StartMapping::StartMapping(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList StartMapping::providedPorts() {
  return {
    BT::InputPort<std::string>("mode", "mapping", "Mapping mode: mapping or slam")
  };
}

BT::NodeStatus StartMapping::tick() {
  auto mode_opt = getInput<std::string>("mode");
  std::string mode = mode_opt ? mode_opt.value() : "mapping";

  config().blackboard->set("is_mapping", true);
  config().blackboard->set("mapping_mode", mode);

  std::shared_ptr<SharedContext> ctx;
  if (config().blackboard->get("context", ctx) && ctx && ctx->node) {
    RCLCPP_INFO(ctx->node->get_logger(), "StartMapping: started mapping node in '%s' mode", mode.c_str());
    // ROS2 Lifecycle client or system/launch integration would go here in Phase 3.
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
