#pragma once

#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>

namespace rover_bt {

class TTSClient {
public:
  TTSClient(rclcpp::Logger logger, const std::string& piper_bin, const std::string& piper_model);
  ~TTSClient();

  void speak(const std::string& text);
  void stop();

private:
  void worker();

  rclcpp::Logger logger_;
  std::string piper_bin_;
  std::string piper_model_;
  bool enabled_ = false;
  std::atomic<bool> stop_flag_{false};
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<std::string> text_queue_;
  std::thread worker_thread_;
};

}  // namespace rover_bt
