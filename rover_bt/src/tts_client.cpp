#include "rover_bt/tts_client.hpp"
#include <iostream>
#include <cstdlib>

namespace rover_bt {

namespace {
// Wrap an arbitrary string in single quotes for safe use in a /bin/sh command,
// escaping any embedded single quotes ( ' → '\'' ).
std::string shell_quote(const std::string& s) {
  std::string out = "'";
  for (char c : s) {
    if (c == '\'') {
      out += "'\\''";
    } else {
      out += c;
    }
  }
  out += "'";
  return out;
}
}  // namespace

TTSClient::TTSClient(rclcpp::Logger logger, const std::string& piper_bin, const std::string& piper_model)
  : logger_(logger), piper_bin_(piper_bin), piper_model_(piper_model) {
  if (!piper_bin_.empty() && !piper_model_.empty()) {
    enabled_ = true;
    worker_thread_ = std::thread(&TTSClient::worker, this);
    RCLCPP_INFO(logger_, "TTSClient initialized. Piper bin: %s, Model: %s", piper_bin_.c_str(), piper_model_.c_str());
  } else {
    RCLCPP_WARN(logger_, "TTSClient disabled: piper_bin or piper_model parameter is empty");
  }
}

TTSClient::~TTSClient() {
  stop();
}

void TTSClient::speak(const std::string& text) {
  if (!enabled_) {
    RCLCPP_INFO(logger_, "[TTS Mock] Speak: %s", text.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(queue_mutex_);
  text_queue_.push(text);
  queue_cv_.notify_one();
}

void TTSClient::stop() {
  if (stop_flag_.exchange(true)) {
    return;
  }
  queue_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void TTSClient::worker() {
  while (!stop_flag_) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait(lock, [this] { return !text_queue_.empty() || stop_flag_; });
    if (stop_flag_) {
      break;
    }
    std::string text = text_queue_.front();
    text_queue_.pop();
    lock.unlock();

    RCLCPP_INFO(logger_, "[TTS] Speaking: %s", text.c_str());

    // Call Piper and pipe to aplay.
    // S16_LE format, 22050Hz sample rate is standard for Piper models.
    // std::system runs the command via /bin/sh (dash on Ubuntu), which does
    // not support the bash here-string operator <<<, so feed the text with a
    // shell-quoted `echo` pipe instead.
    std::string cmd = "echo " + shell_quote(text) + " | " +
                      shell_quote(piper_bin_) + " --model " + shell_quote(piper_model_) +
                      " --output-raw | aplay -r 22050 -f S16_LE -c 1 -q";
    int ret = std::system(cmd.c_str());
    if (ret != 0) {
      RCLCPP_WARN(logger_, "TTS command execution failed or was interrupted");
    }
  }
}

}  // namespace rover_bt
