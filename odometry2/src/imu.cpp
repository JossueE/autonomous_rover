#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class WitmotionImuNode : public rclcpp::Node {
public:
    WitmotionImuNode(const std::string & port = "") : Node("witmotion_imu_node"), serial_fd_(-1) {
        
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<std::string>("frame_id", "imu_link");
        
        if (!port.empty()) {
            port_ = port;
            this->set_parameter(rclcpp::Parameter("port", port));
        } else {
            port_ = this->get_parameter("port").as_string();
        }
        baudrate_ = this->get_parameter("baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        if (!open_serial_port()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s.", port_.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to IMU on %s at %d baud", port_.c_str(), baudrate_);

        // Initialize message with default covariances
        current_imu_msg_.header.frame_id = frame_id_;
        std::fill(current_imu_msg_.orientation_covariance.begin(), current_imu_msg_.orientation_covariance.end(), 0.01);
        std::fill(current_imu_msg_.linear_acceleration_covariance.begin(), current_imu_msg_.linear_acceleration_covariance.end(), 0.01);
        std::fill(current_imu_msg_.angular_velocity_covariance.begin(), current_imu_msg_.angular_velocity_covariance.end(), 0.01);

        timer_ = this->create_wall_timer(10ms, std::bind(&WitmotionImuNode::read_and_publish, this));
    }

    ~WitmotionImuNode() override {
        if (serial_fd_ != -1) {
            close(serial_fd_);
        }
    }

private:
    std::string port_;
    int baudrate_;
    std::string frame_id_;
    int serial_fd_;
    
    std::vector<uint8_t> data_buffer_;
    sensor_msgs::msg::Imu current_imu_msg_;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool open_serial_port() {
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) return false;

        struct termios options;
        tcgetattr(serial_fd_, &options);

        speed_t speed;
        switch (baudrate_) {
            case 9600:   speed = B9600;   break;
            case 115200: speed = B115200; break;
            default:     speed = B115200; break;
        }

        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        options.c_cflag = speed | CS8 | CLOCAL | CREAD;
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        options.c_lflag = 0;

        tcflush(serial_fd_, TCIFLUSH);
        tcsetattr(serial_fd_, TCSANOW, &options);
        return true;
    }

    void parse_packet(uint8_t type, const uint8_t * data) {
        // Equivalent to struct.unpack('<hhhh', payload_data[:8])
        int16_t raw[4];
        for (int i = 0; i < 4; ++i) {
            raw[i] = static_cast<int16_t>((data[2 * i + 1] << 8) | data[2 * i]);
        }

        if (type == 0x51) { // Acceleration
            current_imu_msg_.linear_acceleration.x = (raw[0] / 32768.0) * 16.0 * 9.80665;
            current_imu_msg_.linear_acceleration.y = (raw[1] / 32768.0) * 16.0 * 9.80665;
            current_imu_msg_.linear_acceleration.z = (raw[2] / 32768.0) * 16.0 * 9.80665;
        } 
        else if (type == 0x52) { // Angular Velocity (Gyro)
            current_imu_msg_.angular_velocity.x = (raw[0] / 32768.0) * 2000.0 * (M_PI / 180.0);
            current_imu_msg_.angular_velocity.y = (raw[1] / 32768.0) * 2000.0 * (M_PI / 180.0);
            current_imu_msg_.angular_velocity.z = (raw[2] / 32768.0) * 2000.0 * (M_PI / 180.0);
        }
        else if (type == 0x53) { // Euler Angles -> Orientation
            double roll  = (raw[0] / 32768.0) * 180.0 * (M_PI / 180.0);
            double pitch = (raw[1] / 32768.0) * 180.0 * (M_PI / 180.0);
            double yaw   = (raw[2] / 32768.0) * 180.0 * (M_PI / 180.0);

            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            current_imu_msg_.orientation.x = q.x();
            current_imu_msg_.orientation.y = q.y();
            current_imu_msg_.orientation.z = q.z();
            current_imu_msg_.orientation.w = q.w();
        }
    }

    void read_and_publish() {
        if (serial_fd_ == -1) return;

        uint8_t raw_buffer[1024];
        int bytes_read = read(serial_fd_, raw_buffer, sizeof(raw_buffer));

        if (bytes_read > 0) {
            data_buffer_.insert(data_buffer_.end(), raw_buffer, raw_buffer + bytes_read);

            while (data_buffer_.size() >= 11) {
                if (data_buffer_[0] != 0x55) {
                    data_buffer_.erase(data_buffer_.begin());
                    continue;
                }

                // Checksum is the sum of the first 10 bytes compared with the 11th byte
                uint8_t sum = 0;
                for (int i = 0; i < 10; ++i) {
                    sum += data_buffer_[i];
                }
                if (sum != data_buffer_[10]) {
                    // Checksum failed! Erase the first byte and search for the next 0x55
                    data_buffer_.erase(data_buffer_.begin());
                    continue;
                }

                uint8_t frame_type = data_buffer_[1];
                parse_packet(frame_type, &data_buffer_[2]);

                data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + 11);

                current_imu_msg_.header.stamp = this->get_clock()->now();
                imu_pub_->publish(current_imu_msg_);
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::string port = "";
    if (argc >= 2) {
        port = argv[1];
    }
    rclcpp::spin(std::make_shared<WitmotionImuNode>(port));
    rclcpp::shutdown();
    return 0;
}