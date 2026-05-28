#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <limits>
#include <string>
#include <cmath>

using sensor_msgs::msg::LaserScan;

class PacecatLidarCropNode : public rclcpp::Node
{
public:
  PacecatLidarCropNode()
  : Node("pacecat_lidar_crop_node")
  {
    declare_parameter<std::string>("input_topic", "scan_raw");
    declare_parameter<std::string>("output_topic", "scan");
    declare_parameter<double>("limit_angle", 2.094);  // default 120 degrees
    declare_parameter<bool>("centered", true);
    declare_parameter<double>("pad_value", std::numeric_limits<float>::infinity());

    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("limit_angle", limit_angle_);
    get_parameter("centered", centered_);
    get_parameter("pad_value", pad_value_);

    publisher_ = create_publisher<LaserScan>(output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<LaserScan>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&PacecatLidarCropNode::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "PacecatLidarCropNode ready. input=%s output=%s limit_angle=%.3f centered=%s",
      input_topic_.c_str(), output_topic_.c_str(), limit_angle_, centered_ ? "true" : "false");
  }

private:
  void scan_callback(const LaserScan::SharedPtr msg)
  {
    const size_t count = msg->ranges.size();
    if (count == 0 || msg->angle_increment == 0.0f) {
      RCLCPP_WARN(get_logger(), "Received invalid or empty LaserScan");
      return;
    }

    const double half_limit = limit_angle_ / 2.0;
    const double desired_min = centered_ ? -half_limit : msg->angle_min;
    const double desired_max = centered_ ? half_limit : msg->angle_min + limit_angle_;

    int raw_start = static_cast<int>(std::round((desired_min - msg->angle_min) / msg->angle_increment));
    int raw_end = static_cast<int>(std::round((desired_max - msg->angle_min) / msg->angle_increment));

    raw_start = std::max(0, std::min(raw_start, static_cast<int>(count)));
    raw_end = std::max(0, std::min(raw_end, static_cast<int>(count)));

    if (raw_start > raw_end) {
      std::swap(raw_start, raw_end);
    }

    LaserScan cropped = *msg;
    cropped.ranges.assign(count, static_cast<float>(pad_value_));

    if (cropped.intensities.size() == count) {
      cropped.intensities.assign(count, 0.0f);
    }

    if (raw_start < raw_end) {
      std::copy_n(msg->ranges.begin() + raw_start, raw_end - raw_start, cropped.ranges.begin() + raw_start);
      if (cropped.intensities.size() == count) {
        std::copy_n(msg->intensities.begin() + raw_start, raw_end - raw_start, cropped.intensities.begin() + raw_start);
      }
    }

    cropped.header = msg->header;
    cropped.scan_time = msg->scan_time;
    publisher_->publish(cropped);
  }

  std::string input_topic_;
  std::string output_topic_;
  double limit_angle_;
  bool centered_;
  double pad_value_;

  rclcpp::Subscription<LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PacecatLidarCropNode>());
  rclcpp::shutdown();
  return 0;
}
