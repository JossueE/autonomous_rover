#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <memory>
#include <string>

#include "eyes_gui/srv/set_face_state.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_bt/msg/rover_status.hpp"

namespace
{

std::string toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::string toUpper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return value;
}

}  // namespace

class FaceEmotionController : public rclcpp::Node
{
public:
  FaceEmotionController()
  : rclcpp::Node("face_emotion_controller")
  {
    declare_parameter<std::string>("status_topic", "/rover_bt/status");
    declare_parameter<std::string>("trajectory_topic", "/sdv_trajectory");
    declare_parameter<std::string>("face_state_service", "/robot_face/set_state");
    declare_parameter<double>("blocked_debounce_sec", 0.7);
    declare_parameter<double>("normal_debounce_sec", 0.2);
    declare_parameter<double>("min_trajectory_length_m", 0.10);

    status_topic_ = get_parameter("status_topic").as_string();
    trajectory_topic_ = get_parameter("trajectory_topic").as_string();
    face_state_service_ = get_parameter("face_state_service").as_string();
    blocked_debounce_sec_ = get_parameter("blocked_debounce_sec").as_double();
    normal_debounce_sec_ = get_parameter("normal_debounce_sec").as_double();
    min_trajectory_length_m_ = get_parameter("min_trajectory_length_m").as_double();

    face_client_ = create_client<eyes_gui::srv::SetFaceState>(face_state_service_);

    status_sub_ = create_subscription<rover_bt::msg::RoverStatus>(
      status_topic_, 10,
      [this](const rover_bt::msg::RoverStatus::SharedPtr msg) {
        mode_ = toUpper(msg->mode);
        nav_status_ = toLower(msg->navigation_status);
        have_status_ = true;
        evaluate();
      });

    trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
      trajectory_topic_, 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        trajectory_blocked_ = trajectoryLength(*msg) < min_trajectory_length_m_;
        have_trajectory_ = true;
        evaluate();
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        evaluate();
      });

    RCLCPP_INFO(
      get_logger(),
      "FaceEmotionController watching status='%s', trajectory='%s', service='%s'",
      status_topic_.c_str(), trajectory_topic_.c_str(), face_state_service_.c_str());
  }

private:
  void evaluate()
  {
    if (!have_status_) {
      return;
    }

    const bool autonomous_mode = mode_ == "AUTONOMOUS" || mode_ == "PATROL";
    std::string desired_state = "normal";

    if (autonomous_mode && nav_status_ == "navigating" &&
        have_trajectory_ && trajectory_blocked_) {
      desired_state = "crying";
    }

    requestStateWhenStable(desired_state);
  }

  void requestStateWhenStable(const std::string & state)
  {
    const double now = get_clock()->now().seconds();
    if (candidate_state_ != state) {
      candidate_state_ = state;
      candidate_since_sec_ = now;
    }

    const double debounce =
      state == "crying" ? blocked_debounce_sec_ : normal_debounce_sec_;
    if ((now - candidate_since_sec_) < debounce) {
      return;
    }

    if (last_requested_state_ == state) {
      return;
    }

    sendFaceState(state);
  }

  double trajectoryLength(const nav_msgs::msg::Path & path) const
  {
    if (path.poses.size() < 2) {
      return 0.0;
    }

    double length = 0.0;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto & previous = path.poses[i - 1].pose.position;
      const auto & current = path.poses[i].pose.position;
      length += std::hypot(current.x - previous.x, current.y - previous.y);
    }
    return length;
  }

  void sendFaceState(const std::string & state)
  {
    if (!face_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Face state service '%s' is not available", face_state_service_.c_str());
      return;
    }

    auto request = std::make_shared<eyes_gui::srv::SetFaceState::Request>();
    request->state = state;

    last_requested_state_ = state;
    face_client_->async_send_request(
      request,
      [this, state](rclcpp::Client<eyes_gui::srv::SetFaceState>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_WARN(
              get_logger(), "Face state '%s' rejected: %s",
              state.c_str(), response->message.c_str());
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(
            get_logger(), "Face state '%s' request failed: %s",
            state.c_str(), ex.what());
        }
      });
  }

  std::string status_topic_;
  std::string trajectory_topic_;
  std::string face_state_service_;
  double blocked_debounce_sec_{0.7};
  double normal_debounce_sec_{0.2};
  double min_trajectory_length_m_{0.10};

  bool have_status_{false};
  bool have_trajectory_{false};
  bool trajectory_blocked_{false};
  std::string mode_{"IDLE"};
  std::string nav_status_{"idle"};
  std::string candidate_state_;
  std::string last_requested_state_;
  double candidate_since_sec_{0.0};

  rclcpp::Client<eyes_gui::srv::SetFaceState>::SharedPtr face_client_;
  rclcpp::Subscription<rover_bt::msg::RoverStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaceEmotionController>());
  rclcpp::shutdown();
  return 0;
}
