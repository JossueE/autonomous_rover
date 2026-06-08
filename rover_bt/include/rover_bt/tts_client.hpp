#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace rover_bt {

/**
 * @brief Thin publisher wrapper that turns speech requests into std_msgs/String
 *        messages on a TTS topic for an external speech node (e.g. Piper) to voice.
 */
class TTSClient {
public:
  /**
   * @brief Create the speech publisher.
   * @param node  Node used to create the publisher (not retained beyond construction).
   * @param topic Topic the external TTS engine subscribes to.
   */
  TTSClient(rclcpp::Node* node, rclcpp::Logger logger, const std::string& topic);
  ~TTSClient() = default;

  /// @brief Publish a speech request; logs and no-ops if the publisher is unset.
  void speak(const std::string& text);

private:
  rclcpp::Logger logger_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_pub_;
};

}  // namespace rover_bt
