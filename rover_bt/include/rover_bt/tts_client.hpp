#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace rover_bt {

class TTSClient {
public:
  TTSClient(rclcpp::Node* node, rclcpp::Logger logger, const std::string& topic);
  ~TTSClient() = default;

  void speak(const std::string& text);

private:
  rclcpp::Logger logger_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_pub_;
};

}  // namespace rover_bt
