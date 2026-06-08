#include "rover_bt/tts_client.hpp"

namespace rover_bt {

TTSClient::TTSClient(rclcpp::Node* node, rclcpp::Logger logger, const std::string& topic)
  : logger_(logger) {
  tts_pub_ = node->create_publisher<std_msgs::msg::String>(topic, 10);
  RCLCPP_INFO(logger_, "TTSClient publishing speech requests on: %s", topic.c_str());
}

void TTSClient::speak(const std::string& text) {
  if (!tts_pub_) {
    RCLCPP_INFO(logger_, "[TTS disabled] %s", text.c_str());
    return;
  }
  std_msgs::msg::String msg;
  msg.data = text;
  tts_pub_->publish(msg);
  RCLCPP_INFO(logger_, "[TTS request] %s", text.c_str());
}

}  // namespace rover_bt
