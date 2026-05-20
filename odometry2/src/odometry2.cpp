/* 
@description:
This node computes the odometry of the differential mobile robot, from the count of its encoders, using the class Encoder.
Also, the 'odom' tf is published to get the transformation between the frames 'odom' and 'base_footprint', at 50 Hz.
Robot parameters are required.
The odometry can be reset via the reset_odometry service.

@author: C. Mauricio Arteaga-Escamilla
*/

#include "encoder.h"
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp" //Encoders count message
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
const double pi = 3.141592865358979;


class OdomEncoder : public rclcpp::Node
{
  public:
    OdomEncoder(ENCODER* enco) : Node("odometry2"), enco_(enco)
    {
      ns_ = this->get_namespace();
      if (ns_.size()>1){
        RCLCPP_INFO(this->get_logger(), "The provided ns: %s", ns_.c_str());
        bare_ns_ = ns_.substr(1);
        frame_id_ = bare_ns_+"/odom";
        child_frame_id_ = bare_ns_+"/base_footprint";
      }

      pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("wheel/odom", 5);
      timer_ = this->create_wall_timer(20ms, std::bind(&OdomEncoder::publish_odom, this)); //20 ms = 50Hz
      
      sub_wdata_l = this->create_subscription<std_msgs::msg::Float64>("wheel/left_data", 10, std::bind(&OdomEncoder::leftCallback, this, _1));
      sub_wdata_r = this->create_subscription<std_msgs::msg::Float64>("wheel/right_data", 10, std::bind(&OdomEncoder::rightCallback, this, _1));

      reset_odom_server_ = this->create_service<std_srvs::srv::Trigger>("reset_odometry", std::bind(&OdomEncoder::reset_odom_request, this, _1, _2));

      // Initialize the transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      

      //Get ros params from file (only once)
      this->declare_parameter<bool>("wheelR_is_backward", false);
      get_parameter("wheelR_is_backward", wheelR_is_backward);
      this->declare_parameter<bool>("wheelL_is_backward", true);
      get_parameter("wheelL_is_backward", wheelL_is_backward);
      
      this->declare_parameter<double>("wheels_separation", 0.4);
      get_parameter("wheels_separation", wheels_separation);
      RCLCPP_INFO(this->get_logger(), "wheels_separation: %.3f", wheels_separation);
      
      this->declare_parameter<double>("wheel_radius", 0.1);
      get_parameter("wheel_radius", wheel_radius);
      RCLCPP_INFO(this->get_logger(), "wheel_radius: %.3f", wheel_radius);
      
      this->declare_parameter<double>("TICKS_PER_REV", 4096.0);
      get_parameter("TICKS_PER_REV", TICKS_PER_REV);
      RCLCPP_INFO(this->get_logger(), "TICKS_PER_REV: %.3f", TICKS_PER_REV);
      
      TICKS_PER_METER = TICKS_PER_REV/(2*pi*wheel_radius); //-------------------
      RCLCPP_INFO(this->get_logger(), "TICKS_PER_METER: %.2f", TICKS_PER_METER);
      
      this->declare_parameter<bool>("publish_tf", true);
      get_parameter("publish_tf", publish_tf);
      RCLCPP_INFO(this->get_logger(), "publish_tf: %d", publish_tf);

      RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
      RCLCPP_INFO(this->get_logger(), "child_frame_id: %s", child_frame_id_.c_str());

      left_tic_time_ = now().seconds();
      right_tic_time_ = left_tic_time_;
    }
    
    ~OdomEncoder() //Destructor definition
    {          
      RCLCPP_INFO(this->get_logger(), "Node has been terminated");
    }

    
    float right_tic = 0, left_tic = 0;
    float first_right = 0, first_left = 0;
    double current_time, left_tic_time_, right_tic_time_;
    std::string ns_, bare_ns_ = "", frame_id_ = "odom", child_frame_id_ = "base_footprint";
    
    //ROS params
    bool wheelR_is_backward, wheelL_is_backward, publish_tf;
    double wheels_separation, wheel_radius;
    double TICKS_PER_REV, TICKS_PER_METER; //wheel radius is implicit in TICKS_PER_METER
          
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_wdata_l, sub_wdata_r;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odom_server_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped t; 

  private:
    ENCODER* enco_; // pointer to the ENCODER object
    int number_of_warnings = 0;
    double xy_tolerance_ = 0.5, th_tolerance_ = 0.5;

    void publish_odom()
    {
      current_time = now().seconds();
      if(number_of_warnings <= 3){
        if (current_time - left_tic_time_ > 10){
          RCLCPP_WARN(this->get_logger(), "wheel/left_data topic is not publishing");
          left_tic_time_ = current_time + 65; //This is to report the topic status every 10+65 seconds
          number_of_warnings++;
        }
        if (current_time - right_tic_time_ > 10){
          RCLCPP_WARN(this->get_logger(), "wheel/right_data topic is not publishing");
          right_tic_time_ = current_time +65;
        }
      }

      //This method compute the robot odometry from encoders data
      enco_->computeOdom(wheels_separation, TICKS_PER_METER, current_time);


      //Create and fill out the odom message
      auto odom_msg = nav_msgs::msg::Odometry();
      odom_msg.header.stamp = this->now(); //Get the whole type
      odom_msg.header.frame_id = frame_id_;
      odom_msg.child_frame_id = child_frame_id_;
      odom_msg.pose.pose.position.x = enco_->x;
      odom_msg.pose.pose.position.y = enco_->y;

      //Compute the quaternion from Euler angles
      tf2::Quaternion q;
      q.setRPY(0,0,enco_->th);
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();

      //Set pose covariances
      odom_msg.pose.covariance[0] = 0.05; //covariance_x
      odom_msg.pose.covariance[7] = 0.05; //covariance_y
      odom_msg.pose.covariance[35] = 0.1; //covariance_th

      //Robot velocities are filtered in the class
      odom_msg.twist.twist.linear.x = enco_->vx;    //odom_msg.twist.twist.linear.y = 0;
      odom_msg.twist.twist.angular.z = enco_->vth;
      //Twist has its own covariance
      odom_msg.twist.covariance[0] = 0.04;
      odom_msg.twist.covariance[35] = 0.005;

      //Create and fill out the transform broadcaster
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = frame_id_;
      t.child_frame_id = child_frame_id_;

      t.transform.translation.x = enco_->x;
      t.transform.translation.y = enco_->y;

      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      pub_odom->publish(odom_msg); //Publish wheel odometry
      if(publish_tf) tf_broadcaster_->sendTransform(t); // Send the transformation
      
    }
    
    void leftCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
      left_tic = msg->data - first_left;
      if (wheelL_is_backward) left_tic = (-1*left_tic);
      enco_->lefttick(left_tic);
      left_tic_time_ = now().seconds();
    }

    void rightCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
      right_tic = msg->data - first_right;
      if (wheelR_is_backward) right_tic = (-1*right_tic);
      enco_->righttick(right_tic);
      right_tic_time_ = now().seconds();
    }

    void reset_odom_request(const std_srvs::srv::Trigger::Request::SharedPtr request_message, std_srvs::srv::Trigger::Response::SharedPtr response_message)
    {
      (void)request_message;
      double robot_pos = enco_->xy_pos;
      if(robot_pos > xy_tolerance_){
        RCLCPP_WARN(this->get_logger(), "Robot not at home due to xy position: %.2f m", robot_pos);
      }
      if( abs(enco_->th) > th_tolerance_){
        RCLCPP_WARN(this->get_logger(), "Robot not at home due to orientation: %.2f rad", enco_->th);
      }
      enco_->reset_odometry();
      response_message->success = true;
      // RCLCPP_INFO(this->get_logger(), "Odometry was reset");
      response_message->message = "Odometry was reset";
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  ENCODER enco; // create an instance of the ENCODER class
  auto node = std::make_shared<OdomEncoder>(&enco); // pass a pointer to the instance to the OdomEncoder constructor
    
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


