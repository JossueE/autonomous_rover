#ifndef EYES_GUI__FACE_STATE_NODE_HPP_
#define EYES_GUI__FACE_STATE_NODE_HPP_

#include <memory>
#include <set>
#include <string>

#include <QObject>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "eyes_gui/srv/set_face_state.hpp"

namespace eyes_gui
{

class FaceStateNode : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  FaceStateNode();

public Q_SLOTS:
  void publishCurrentState(const QString & state);

Q_SIGNALS:
  void stateCommandReceived(const QString & state);

private:
  using SetFaceState = eyes_gui::srv::SetFaceState;

  bool isAcceptedState(const std::string & state) const;

  rclcpp::Service<SetFaceState>::SharedPtr set_state_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_state_publisher_;
  std::string last_published_state_;
  std::set<std::string> accepted_states_;
};

}  // namespace eyes_gui

#endif  // EYES_GUI__FACE_STATE_NODE_HPP_
