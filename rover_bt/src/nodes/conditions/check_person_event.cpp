#include "rover_bt/nodes/conditions/check_person_event.hpp"
#include "rover_bt/shared_context.hpp"

namespace rover_bt {

CheckPersonEvent::CheckPersonEvent(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList CheckPersonEvent::providedPorts() {
  return {
    BT::InputPort<std::string>("event", "Which edge to consume: lost or found")
  };
}

BT::NodeStatus CheckPersonEvent::tick() {
  auto event = getInput<std::string>("event");
  if (!event) {
    return BT::NodeStatus::FAILURE;
  }

  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx) {
    return BT::NodeStatus::FAILURE;
  }

  std::atomic<bool>* flag = nullptr;
  if (event.value() == "lost") {
    flag = &ctx->person_lost_event;
  } else if (event.value() == "found") {
    flag = &ctx->person_found_event;
  } else {
    return BT::NodeStatus::FAILURE;
  }

  // Consume the one-shot event: SUCCESS only on the tick it was set.
  bool expected = true;
  if (flag->compare_exchange_strong(expected, false)) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace rover_bt
