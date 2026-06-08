#pragma once

#include <mutex>
#include <queue>
#include <optional>
#include <vector>

#include "rover_bt/msg/command.hpp"

namespace rover_bt {

struct PriorityCompare {
  bool operator()(const rover_bt::msg::Command& a, const rover_bt::msg::Command& b) const {
    return a.priority > b.priority;  // Lower number = higher priority
  }
};

class CommandArbitrator {
public:
  CommandArbitrator() = default;
  ~CommandArbitrator() = default;

  void enqueue(const rover_bt::msg::Command& cmd);
  std::optional<rover_bt::msg::Command> consume();
  void clear();

private:
  std::priority_queue<rover_bt::msg::Command, std::vector<rover_bt::msg::Command>, PriorityCompare> queue_;
  std::mutex mutex_;
};

}  // namespace rover_bt
