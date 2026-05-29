#include "rover_bt/nodes/actions/save_location.hpp"
#include "rover_bt/shared_context.hpp"
#include "rover_bt/location_registry.hpp"

namespace rover_bt {

SaveLocation::SaveLocation(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList SaveLocation::providedPorts() {
  return {
    BT::InputPort<std::string>("location_name", "Name of the location to save")
  };
}

BT::NodeStatus SaveLocation::tick() {
  auto name_opt = getInput<std::string>("location_name");
  if (!name_opt) {
    return BT::NodeStatus::FAILURE;
  }

  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->node || !ctx->location_registry) {
    return BT::NodeStatus::FAILURE;
  }

  std::string loc_name = name_opt.value();
  double x = ctx->robot_x.load();
  double y = ctx->robot_y.load();
  double theta = ctx->robot_theta.load();

  bool success = ctx->location_registry->save(loc_name, x, y, theta);
  if (success) {
    RCLCPP_INFO(ctx->node->get_logger(), "SaveLocation: Saved current pose as '%s' at (%.2f, %.2f, %.2f)",
                loc_name.c_str(), x, y, theta);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(ctx->node->get_logger(), "SaveLocation: Failed to save location '%s'", loc_name.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rover_bt
