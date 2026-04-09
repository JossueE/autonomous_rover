#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>  // For access() and F_OK
#include <fcntl.h>   // For file control options
#include <termios.h> // For serial port control

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

std::string node_name = "witmotion_imu_node";

class WitmotionImuNode : public rclcpp::Node {
public:
    WitmotionImuNode(const std::string & port) : Node(node_name), port_(port), serial_fd_(-1) {
        
        // 1. Declare and read parameters
        this->declare_parameter<int>("baudrate", 9600);
        this->declare_parameter<std::string>("frame_id", "imu_link");
        
        baudrate_ = this->get_parameter("baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // 2. Setup Publisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        // 3. Hardware connection attempt
        if (!open_serial_port()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s. Node will shut down.", port_.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully connected to IMU on port %s", port_.c_str());

        // 4. Timer setup (10ms = 100Hz) matching the project's non-blocking style
        timer_ = this->create_wall_timer(10ms, std::bind(&WitmotionImuNode::read_and_publish, this));
    }

    ~WitmotionImuNode() override {
        // Safe shutdown process
        if (serial_fd_ != -1) {
            close(serial_fd_);
            RCLCPP_INFO(this->get_logger(), "Serial port closed safely.");
        }
    }

private:
    std::string port_;
    int baudrate_;
    std::string frame_id_;
    int serial_fd_; // File descriptor for the hardware connection
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool open_serial_port() {
        // Open port in read/write mode, no terminal control, non-blocking
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            return false;
        }
        
        struct termios options;
        tcgetattr(serial_fd_, &options);
        
        // Standard configuration for raw serial data (8N1)
        options.c_cflag = B9600 | CS8 | CLOCAL | CREAD; 
        options.c_iflag = IGNPAR; // Ignore framing errors and parity errors
        options.c_oflag = 0;
        options.c_lflag = 0;
        
        tcflush(serial_fd_, TCIFLUSH);
        tcsetattr(serial_fd_, TCSANOW, &options);
        return true;
    }

    void read_and_publish() {
        if (serial_fd_ == -1) return;

        unsigned char buffer[256];
        // Read available bytes without blocking the ROS 2 executor
        int bytes_read = read(serial_fd_, buffer, sizeof(buffer));

        if (bytes_read > 0) {
            // Create the ROS 2 message
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = this->get_clock()->now();
            imu_msg.header.frame_id = frame_id_;
            
            // TODO: Parse the 'buffer' array looking for the 0x55 headers
            // and populate the imu_msg fields. 
            // Example structure:
            // imu_msg.linear_acceleration.x = parsed_acc_x;
            // imu_msg.angular_velocity.z = parsed_gyro_z;
            
            // Covariances setup (essential for robot_localization integration)
            imu_msg.orientation_covariance[0] = 0.01;
            imu_msg.linear_acceleration_covariance[0] = 0.01;
            imu_msg.angular_velocity_covariance[0] = 0.01;

            imu_pub_->publish(imu_msg);

        } else if (bytes_read < 0 && errno != EAGAIN) {
            // Log if there is a real physical disconnection, ignore if just waiting
            RCLCPP_WARN(this->get_logger(), "Serial read error or device disconnected.");
        }
    }
};

int main(int argc, char * argv[]) {
    auto logger = rclcpp::get_logger(node_name);
    std::string port = "";

    // Argument validation matching wheels_driver.cpp standard
    if (argc >= 2) {
        port = argv[1];
        if (access(port.c_str(), F_OK) != 0) {
            RCLCPP_ERROR(logger, "Port %s does not exist. Please check the connection.", port.c_str());
            return 1;
        }
    } else {
        RCLCPP_ERROR(logger, "No port provided. Usage: ros2 run <package> <node> /dev/ttyUSBX");
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WitmotionImuNode>(port));
    rclcpp::shutdown();
    return 0;
}