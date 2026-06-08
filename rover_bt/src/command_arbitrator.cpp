#include "rover_bt/command_arbitrator.hpp"

namespace rover_bt {

void CommandArbitrator::enqueue(const rover_bt::msg::Command& cmd) {
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.push(cmd);
}

std::optional<rover_bt::msg::Command> CommandArbitrator::consume() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty()) {
    return std::nullopt;
  }
  auto cmd = queue_.top();
  queue_.pop();
  return cmd;
}

void CommandArbitrator::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::priority_queue<rover_bt::msg::Command, std::vector<rover_bt::msg::Command>, PriorityCompare> empty_queue;
  std::swap(queue_, empty_queue);
}

}  // namespace rover_bt
