#pragma once

#include <mutex>
#include <queue>
#include <optional>
#include <vector>

#include "rover_bt/msg/command.hpp"

namespace rover_bt {

/// Orders commands so the smallest `priority` value is served first (a min-heap
/// over priority: 0 = emergency_stop, higher numbers = lower urgency).
struct PriorityCompare {
  bool operator()(const rover_bt::msg::Command& a, const rover_bt::msg::Command& b) const {
    return a.priority > b.priority;  // Lower number = higher priority
  }
};

/**
 * @brief Thread-safe priority inbox for commands arriving from multiple sources
 *        (voice, joycon, GUI, service) on different threads.
 *
 * ROS callbacks enqueue concurrently while the BT's ProcessCommand consumes one
 * command per tick; an internal mutex serialises both. Highest-priority
 * (lowest-numbered) command is always served first, so an emergency_stop jumps
 * ahead of queued lower-priority work.
 */
class CommandArbitrator {
public:
  CommandArbitrator() = default;
  ~CommandArbitrator() = default;

  /// @brief Push a command into the priority queue (callable from any thread).
  void enqueue(const rover_bt::msg::Command& cmd);

  /**
   * @brief Pop the highest-priority command.
   * @return The command, or std::nullopt if the queue is empty.
   */
  std::optional<rover_bt::msg::Command> consume();

  /// @brief Drop all queued commands (e.g. when entering EMERGENCY).
  void clear();

private:
  std::priority_queue<rover_bt::msg::Command, std::vector<rover_bt::msg::Command>, PriorityCompare> queue_;
  std::mutex mutex_;
};

}  // namespace rover_bt
