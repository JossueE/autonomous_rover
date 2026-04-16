#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;

class OdometryFusionNode : public rclcpp::Node {
public:
    OdometryFusionNode() : Node("odometry_fusion_node"),
        x_(0.0), y_(0.0), theta_(0.0), fused_velocity_(0.0), encoder_velocity_(0.0) 
    {
        // Parameters
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<double>("fusion_alpha", 0.85); 

        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        alpha_ = this->get_parameter("fusion_alpha").as_double();

        // Publishers and Broadcasters
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/fused", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribers
        // Assuming encoders publish their calculated linear velocity here
        encoder_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "encoder/velocity", 10, std::bind(&OdometryFusionNode::encoder_callback, this, _1));
            
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&OdometryFusionNode::imu_callback, this, _1));

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Odometry Fusion Node started.");
    }

private:
    std::string odom_frame_;
    std::string base_frame_;
    
    // State variables
    double x_;
    double y_;
    double theta_;
    double fused_velocity_;
    double encoder_velocity_;
    double alpha_; // Weight for Complementary Filter
    
    rclcpp::Time last_time_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    void encoder_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Store the latest linear velocity reported by the wheels
        encoder_velocity_ = msg->linear.x;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time_).seconds();
        
        if (dt <= 0.0) return; // Prevent division by zero or negative time
        last_time_ = current_time;

        // 1. Extract Heading (Yaw) directly from IMU orientation
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        theta_ = yaw;

        // 2. Sensor Fusion for Linear Velocity (Complementary Filter)
        // V_imu = previous_velocity + acceleration * dt
        double imu_acceleration_x = msg->linear_acceleration.x;
        double velocity_from_imu = fused_velocity_ + (imu_acceleration_x * dt);
        
        // Fused V = alpha * V_encoder + (1 - alpha) * V_imu
        fused_velocity_ = (alpha_ * encoder_velocity_) + ((1.0 - alpha_) * velocity_from_imu);

        // 3. Kinematic Integration for Position
        double delta_x = fused_velocity_ * cos(theta_) * dt;
        double delta_y = fused_velocity_ * sin(theta_) * dt;

        x_ += delta_x;
        y_ += delta_y;

        publish_odometry(current_time, q);
    }

    void publish_odometry(const rclcpp::Time & current_time, const tf2::Quaternion & current_quat) {
        // Broadcast TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = odom_frame_;
        t.child_frame_id = base_frame_;

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = current_quat.x();
        t.transform.rotation.y = current_quat.y();
        t.transform.rotation.z = current_quat.z();
        t.transform.rotation.w = current_quat.w();

        tf_broadcaster_->sendTransform(t);

        // Publish Odometry message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        
        odom.pose.pose.orientation.x = current_quat.x();
        odom.pose.pose.orientation.y = current_quat.y();
        odom.pose.pose.orientation.z = current_quat.z();
        odom.pose.pose.orientation.w = current_quat.w();

        odom.twist.twist.linear.x = fused_velocity_;
        odom.twist.twist.angular.z = 0.0; // Handled purely by orientation update in this model

        odom_pub_->publish(odom);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryFusionNode>());
    rclcpp::shutdown();
    return 0;
}