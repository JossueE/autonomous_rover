#include "eyes_gui/face_state_node.hpp"

#include <algorithm>
#include <cctype>

namespace eyes_gui
{

FaceStateNode::FaceStateNode()
: QObject(),
  rclcpp::Node("robot_face_state_node", rclcpp::NodeOptions().use_global_arguments(false)),
  accepted_states_({"normal", "recover", "crying", "debug", "face", "fullscreen", "blink"})
{
  current_state_publisher_ = create_publisher<std_msgs::msg::String>(
    "/robot_face/current_state", rclcpp::QoS(1).transient_local().reliable());

  set_state_service_ = create_service<SetFaceState>(
    "/robot_face/set_state",
    [this](
      const std::shared_ptr<SetFaceState::Request> request,
      std::shared_ptr<SetFaceState::Response> response)
    {
      std::string state = request->state;
      std::transform(state.begin(), state.end(), state.begin(), [](unsigned char value) {
        return static_cast<char>(std::tolower(value));
      });

      if (!isAcceptedState(state)) {
        response->success = false;
        response->message = "Unsupported state: " + request->state;
        return;
      }

      response->success = true;
      response->message = "Accepted state: " + state;
      Q_EMIT stateCommandReceived(QString::fromStdString(state));
    });
}

void FaceStateNode::publishCurrentState(const QString & state)
{
  const std::string value = state.toStdString();
  if (value.empty() || value == last_published_state_) {
    return;
  }

  if (value != "normal" && value != "crying" && value != "debug") {
    return;
  }

  std_msgs::msg::String message;
  message.data = value;
  current_state_publisher_->publish(message);
  last_published_state_ = value;
}

bool FaceStateNode::isAcceptedState(const std::string & state) const
{
  return accepted_states_.find(state) != accepted_states_.end();
}

}  // namespace eyes_gui
