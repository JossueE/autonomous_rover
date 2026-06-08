#include "rover_bt/nodes/actions/set_person_profile.hpp"
#include "rover_bt/shared_context.hpp"

#include <algorithm>
#include <cctype>

namespace rover_bt {

SetPersonProfile::SetPersonProfile(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList SetPersonProfile::providedPorts() {
  return {
    BT::InputPort<std::string>("profile", "Tuning profile to push: indoor or outdoor")
  };
}

namespace {
std::string trim_lower(std::string s) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return s;
}
}  // namespace

BT::NodeStatus SetPersonProfile::tick() {
  std::string profile;
  if (auto p = getInput<std::string>("profile")) {
    profile = trim_lower(p.value());
  }

  // Empty target → keep the tracker's current profile (no message).
  if (profile.empty()) {
    return BT::NodeStatus::SUCCESS;
  }

  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->person_profile_pub) {
    return BT::NodeStatus::FAILURE;
  }

  if (profile != "indoor" && profile != "outdoor") {
    if (ctx->node) {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "SetPersonProfile: unknown profile '%s' (expected indoor/outdoor); ignoring.",
                  profile.c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }

  std_msgs::msg::String msg;
  msg.data = profile;
  ctx->person_profile_pub->publish(msg);

  if (ctx->node) {
    RCLCPP_INFO(ctx->node->get_logger(),
                "SetPersonProfile: switched person_tracker to '%s'.", profile.c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
