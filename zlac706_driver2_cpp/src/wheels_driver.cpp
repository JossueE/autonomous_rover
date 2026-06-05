#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "wheels_writer.h"
#include "zlac706_driver.h"

using namespace std::chrono_literals;

std::string node_name = "wheels_driver";

constexpr int wait_time{10};

// This driver is for ZLAC706
class ZlacNode : public rclcpp::Node {
public:
  ZlacNode(const std::string &port_left, const std::string &port_right)
      : Node(node_name), port_left_(port_left), port_right_(port_right) {

    // Serial Parameters only for ZLAC706
    this->declare_parameter<int>("rate", 10);
    this->declare_parameter<int>("can_id", 666);
    this->declare_parameter<int>("baudrate", 57600);

    // Wheels configuration parameters
    this->declare_parameter<bool>("wheelR_is_backward", true);
    this->declare_parameter<bool>("wheelL_is_backward", false);
    this->declare_parameter<double>("wheels_separation", 0.315);
    this->declare_parameter<double>("wheel_radius", 0.1f);

    // Acceleration and deceleration time for velocity control mode.
    // The driver can receive accel for left and right wheel separately.
    this->declare_parameter<int>("accel_time_ms", 1);
    this->declare_parameter<int>("decel_time_ms", 1);

    // Driver Movement Lock
    // If "unlock_driver" == true, after "time_disabled_driver_s":
    //     The wheels will be unlocked and current will be limited to 3A.
    // If "unlock_driver" == false, after "time_disabled_driver_s":
    //     The wheels will be locked and current will be limited to 3A.
    this->declare_parameter<bool>("unlock_driver", true);
    this->declare_parameter<double>("time_disabled_driver_s", 3.0f);

    // PID gains for velocity control mode
    this->declare_parameter<std::vector<int64_t>>(
        "left_velocity_gains", std::vector<int64_t>{6000, 200, 1000});
    this->declare_parameter<std::vector<int64_t>>(
        "right_velocity_gains", std::vector<int64_t>{6000, 200, 1000});

    // Serial Parameters only for ZLAC706
    rate_ = this->get_parameter("rate").as_int();
    can_id_ = this->get_parameter("can_id").as_int();
    baudrate_ = this->get_parameter("baudrate").as_int();

    // Wheels configuration parameters
    wheelR_is_backward_ = this->get_parameter("wheelR_is_backward").as_bool();
    wheelL_is_backward_ = this->get_parameter("wheelL_is_backward").as_bool();
    wheels_separation_ = this->get_parameter("wheels_separation").as_double();
    wheels_radius_ = this->get_parameter("wheel_radius").as_double();

    // Acceleration and deceleration time for velocity control mode
    accel_time_ms_ = this->get_parameter("accel_time_ms").as_int();
    decel_time_ms_ = this->get_parameter("decel_time_ms").as_int();

    // Driver Movement Lock
    unlock_driver_ = this->get_parameter("unlock_driver").as_bool();
    time_disabled_driver_s_ =
        get_parameter("time_disabled_driver_s").as_double();

    // PID gains for velocity control mode
    auto v_l = this->get_parameter("left_velocity_gains").as_integer_array();
    auto v_r = this->get_parameter("right_velocity_gains").as_integer_array();

    velocity_gains_.left.kp = static_cast<int16_t>(v_l[0]);
    velocity_gains_.left.ki = static_cast<int16_t>(v_l[1]);
    velocity_gains_.left.kf = static_cast<int16_t>(v_l[2]);
    velocity_gains_.right.kp = static_cast<int16_t>(v_r[0]);
    velocity_gains_.right.ki = static_cast<int16_t>(v_r[1]);
    velocity_gains_.right.kf = static_cast<int16_t>(v_r[2]);

    // -------------------------------------- Timers to Manage the Driver
    // Control ----------------------------------------
    warning_timer_ = this->create_wall_timer(
        1000ms, std::bind(&ZlacNode::warning_handler, this));
        
    wheel_ticks_timer_ = this->create_wall_timer(
        20ms, std::bind(&ZlacNode::wheel_ticks_timer, this));

    movement_lock_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(time_disabled_driver_s_)),
        std::bind(&ZlacNode::movement_lock_timer, this));
    movement_lock_timer_->cancel();

    // ------------------------------------------- Publishers and Subscribers
    // --------------------------------------------
    pub_left_data_ = this->create_publisher<std_msgs::msg::Float64>(
        "wheel/left_data", 10); // Recommendation: Send both in one message
    pub_right_data_ =
        this->create_publisher<std_msgs::msg::Float64>("wheel/right_data", 10);
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&ZlacNode::command_vel_CB, this, std::placeholders::_1));

    // ----------------------------------------------- Parameter Callback
    // ------------------------------------------------

    params_handler_ = this->add_on_set_parameters_callback(
        std::bind(&ZlacNode::on_params_change, this, std::placeholders::_1));

    // ----------------------------------------- Initialization of Motor Control
    // -----------------------------------------

    // Start motor connection
    motor_left_ = ZLAC706();
    motor_right_ = ZLAC706();

    motor_left_.port = port_left_;
    motor_right_.port = port_right_;

    motor_left_.baudrate = baudrate_;
    motor_right_.baudrate = baudrate_;

    // Get devices
    std::vector<std::string> devs_left, devs_right;
    motor_left_.getDevices(devs_left);
    motor_right_.getDevices(devs_right);

    // Start Serial
    serial_left_.open(motor_left_.port, motor_left_.baudrate);
    serial_right_.open(motor_right_.port, motor_right_.baudrate);

    serial_left_.setCallback([this](const char *data, size_t len) {
      motor_left_.received(data, static_cast<unsigned int>(len));
    });
    serial_right_.setCallback([this](const char *data, size_t len) {
      motor_right_.received(data, static_cast<unsigned int>(len));
    });

    // ------------------------------
    motor_left_.getParams();
    serial_left_.write(motor_left_.gParams_msg);

    motor_right_.getParams();
    serial_right_.write(motor_right_.gParams_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time));

    motor_left_.setSpeedMode();
    serial_left_.write(motor_left_.spm_msg);

    motor_right_.setSpeedMode();
    serial_right_.write(motor_right_.spm_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time));

    // Set the acceleration and deceleration time for velocity control mode
    motor_left_.configAcc(
        ms_to_zlac_units(accel_time_ms_, "Acceleration time"),
        ms_to_zlac_units(decel_time_ms_, "Deceleration time"));
    serial_left_.write(motor_left_.cfg_msg);

    motor_right_.configAcc(
        ms_to_zlac_units(accel_time_ms_, "Acceleration time"),
        ms_to_zlac_units(decel_time_ms_, "Deceleration time"));
    serial_right_.write(motor_right_.cfg_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time));

    // Set the PID gains for velocity control mode
    motor_left_.setGain(3, velocity_gains_.left.kp);
    serial_left_.write(motor_left_.kpG_msg);

    motor_right_.setGain(3, velocity_gains_.right.kp);
    serial_right_.write(motor_right_.kpG_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time));

    motor_left_.setGain(2, velocity_gains_.left.kf);
    serial_left_.write(motor_left_.kdG_msg);

    motor_right_.setGain(2, velocity_gains_.right.kf);
    serial_right_.write(motor_right_.kdG_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time));

    motor_left_.setGain(1, velocity_gains_.left.ki);
    serial_left_.write(motor_left_.kiG_msg);

    motor_right_.setGain(1, velocity_gains_.right.ki);
    serial_right_.write(motor_right_.kiG_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time));

    // Set speed to 0 m/s
    motor_left_.setSpeed(0);
    serial_left_.write(motor_left_.sSpeed_msg);

    motor_right_.setSpeed(0);
    serial_right_.write(motor_right_.sSpeed_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time));

    // Start the motor
    motor_left_.start();
    serial_left_.write(motor_left_.str_msg);

    motor_right_.start();
    serial_right_.write(motor_right_.str_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(wait_time));

    left_writer_ = std::make_unique<WheelWriter>(motor_left_, serial_left_);
    right_writer_ = std::make_unique<WheelWriter>(motor_right_, serial_right_);

    left_writer_->start();
    right_writer_->start();

    RCLCPP_INFO(rclcpp::get_logger(node_name), "Node initialized");
  }

  ~ZlacNode() override {
    try {
      left_writer_->request_speed(0.0);
      right_writer_->request_speed(0.0);
      rclcpp::sleep_for(std::chrono::milliseconds(wait_time));

      if (left_writer_) {
        left_writer_->stop_thread();
      }
      if (right_writer_) {
        right_writer_->stop_thread();
      }
      serial_left_.clearCallback();
      serial_right_.clearCallback();

    } catch (...) {
      RCLCPP_ERROR(
          rclcpp::get_logger(node_name),
          "Error launching emergency stop when the node was destructed");
    }
  }

private:
  // Error and troubleshooting
  rclcpp::TimerBase::SharedPtr warning_timer_;

  // Encoder - Wheels Ticks
  rclcpp::TimerBase::SharedPtr wheel_ticks_timer_;
  rclcpp::TimerBase::SharedPtr movement_lock_timer_;

  // Publishers and Subscribers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_data_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_data_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  // Params Callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      params_handler_;

  // Serial Parameters only for ZLAC706
  int rate_;
  int can_id_;
  int baudrate_;
  CallbackAsyncSerial serial_left_;
  CallbackAsyncSerial serial_right_;
  ZLAC706 motor_left_;
  ZLAC706 motor_right_;

  // Thread to control the motors
  std::unique_ptr<WheelWriter> left_writer_;
  std::unique_ptr<WheelWriter> right_writer_;
  bool idle_pending_{false};

  bool wheelR_is_backward_;
  bool wheelL_is_backward_;
  double wheels_separation_;
  double wheels_radius_;
  int accel_time_ms_;
  int decel_time_ms_;
  bool unlock_driver_;
  double time_disabled_driver_s_;
  VelocityGains velocity_gains_;
  const std::string port_left_;
  const std::string port_right_;

  static uint8_t ms_to_zlac_units(int ms, const std::string &label) {
    constexpr int kMinMs = 100;
    constexpr int kMaxMs = 25500; // 255 units * 100 ms

    int clamped_ms = ms;

    if (ms < kMinMs) {
      RCLCPP_WARN(
          rclcpp::get_logger(node_name),
          "%s = %d ms is too low for ZLAC706. Minimum is %d ms. Using %d ms.",
          label.c_str(), ms, kMinMs, kMinMs);
      clamped_ms = kMinMs;
    } else if (ms > kMaxMs) {
      RCLCPP_WARN(
          rclcpp::get_logger(node_name),
          "%s = %d ms is too high for ZLAC706. Maximum is %d ms. Using %d ms.",
          label.c_str(), ms, kMaxMs, kMaxMs);
      clamped_ms = kMaxMs;
    }

    const int units = clamped_ms / 100;

    RCLCPP_INFO(rclcpp::get_logger(node_name), "%s: %d ms -> %d units",
                label.c_str(), clamped_ms, units);

    return static_cast<uint8_t>(units);
  }

  rcl_interfaces::msg::SetParametersResult
  on_params_change(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Param Accepted";

    for (const auto &p : params) {
      if (p.get_name() == "unlock_driver") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "unlock_driver must be bool (true/false)";
          return result;
        }
        unlock_driver_ = p.as_bool();
        movement_lock_timer_->reset();
        RCLCPP_INFO(rclcpp::get_logger(node_name),
                    "unlock_driver setted as: %s",
                    unlock_driver_ ? "true" : "false");
      }
    }
    return result;
  }

  void warning_handler() {

    // If more than one publisher
    const size_t pubs = count_publishers("cmd_vel");
    if (pubs > 1) {
      RCLCPP_FATAL(rclcpp::get_logger(node_name),
                   "More than one publisher on cmd_vel (%zu). Stopping "
                   "robot and shutting down.",
                   pubs);
      // Set speed to 0 m/s
      left_writer_->request_speed(0.0);
      right_writer_->request_speed(0.0);
      // Destroy node
      rclcpp::shutdown();
    } else if (pubs == 0) {
      RCLCPP_WARN(rclcpp::get_logger(node_name),
                  "No publishers on cmd_vel (%zu). Stopping robot and "
                  "locking wheels for security.",
                  pubs);
      // Set speed to 0 m/s
      left_writer_->request_speed(0.0);
      right_writer_->request_speed(0.0);
    }
  }

  void wheel_ticks_timer() {
    left_writer_->request_encoder();
    right_writer_->request_encoder();

    std_msgs::msg::Float64 msg_left_, msg_right_;

    msg_left_.data = static_cast<double>(motor_left_.ticks);
    msg_right_.data = static_cast<double>(motor_right_.ticks);

    pub_left_data_->publish(msg_left_);
    pub_right_data_->publish(msg_right_);
  }

  void command_vel_CB(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const double vx = msg->linear.x;
    const double wz = msg->angular.z;

    RCLCPP_INFO(rclcpp::get_logger(node_name),
                "Angular Speed: %.1f, Linear Speed: %.1f", wz, vx);

    if (vx == 0.0 && wz == 0.0) {
      if (!idle_pending_) {
        idle_pending_ = true;
        movement_lock_timer_->reset();
      }
    } else {
      if (idle_pending_) {
        movement_lock_timer_->cancel();

        left_writer_->request_start();
        right_writer_->request_start();

        idle_pending_ = false;
      }
    }

    auto [rpm_l, rpm_r] = twist_to_rpm(vx, wz);

    RCLCPP_INFO(rclcpp::get_logger(node_name),
                "Publish Speed RPM LEFT: %.1f, RIGHT: %.1f", rpm_l, rpm_r);

    left_writer_->request_speed(rpm_l);
    right_writer_->request_speed(rpm_r);
  }

  void movement_lock_timer() {
    if (!unlock_driver_) {
      left_writer_->request_start();
      right_writer_->request_start();

      left_writer_->request_speed(0);
      right_writer_->request_speed(0);
      RCLCPP_DEBUG(rclcpp::get_logger(node_name),
                   "Driver locked, free wheels disabled");
    } else {
      left_writer_->request_stop();
      right_writer_->request_stop();
      RCLCPP_DEBUG(rclcpp::get_logger(node_name), "Driver unlocked");
    }

    movement_lock_timer_->cancel();
  }

  std::pair<float, float> twist_to_rpm(double vx, double wz) {
    const double w_l = (vx - (wz * wheels_separation_ / 2.0)) / wheels_radius_;
    const double w_r = (vx + (wz * wheels_separation_ / 2.0)) / wheels_radius_;

    double rpm_l = w_l * 60.0 / (2.0 * M_PI);
    double rpm_r = w_r * 60.0 / (2.0 * M_PI);

    if (wheelL_is_backward_)
      rpm_l = -rpm_l;
    if (wheelR_is_backward_)
      rpm_r = -rpm_r;

    rpm_l = static_cast<float>(rpm_l);
    rpm_r = static_cast<float>(rpm_r);

    return {rpm_l, rpm_r};
  }
};

int main(int argc, char *argv[]) {
  auto logger = rclcpp::get_logger(node_name);

  std::string port_left = "";
  std::string port_right = "";

  if (argc >= 3) {
    port_left = argv[1];
    port_right = argv[2];

    // Validate if port_left exists
    if (access(port_left.c_str(), F_OK) != 0) {
      RCLCPP_ERROR(logger, "port_left %s does not exist", port_left.c_str());
      return 1;
    }
    if (access(port_right.c_str(), F_OK) != 0) {
      RCLCPP_ERROR(logger, "port_right %s does not exist", port_right.c_str());
      return 1;
    }
    RCLCPP_INFO(logger, "port_left: %s", port_left.c_str());
    RCLCPP_INFO(logger, "port_right: %s", port_right.c_str());
  } else {
    RCLCPP_ERROR(logger, "No argument given. Usage: \n$ ros2 run "
                         "<package_name> <node_name> <port_left>");
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZlacNode>(port_left, port_right));
  rclcpp::shutdown();
  return 0;
}
