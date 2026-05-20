/* 
@description:
This node creates a transformation listener to get the relative position and rotation between 
'map' and 'base_footprint' frames, given by the user via arguments.
Also, the position and orientation are published on the '/amcl_robot_pose' topic, at 20 Hz, using the Pose message type.

@author: C. Mauricio Arteaga-Escamilla

*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
std::string map_frame = "map", robot_footprint_frame = "base_footprint", node_name = "amcl_robot_pose";


class AmclRobotPose : public rclcpp::Node
{
  public:
    AmclRobotPose() : Node(node_name) {
      
      std::string ns = this->get_namespace();
      if (ns.size()>1){
        RCLCPP_INFO(this->get_logger(), "The provided ns: %s", ns.c_str());
        std::string bare_ns = ns.substr(1);
        map_frame = bare_ns+"/"+map_frame;
        robot_footprint_frame = bare_ns+"/"+robot_footprint_frame;
      }

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
      publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("amcl_robot_pose", 10);
      timer_ = this->create_wall_timer(200ms, std::bind(&AmclRobotPose::on_timer, this));
      
      RCLCPP_INFO(this->get_logger(), "Node initialized");
  }
  
  ~AmclRobotPose() { RCLCPP_INFO(this->get_logger(), "Node finished"); }

private:
  void on_timer() {    
    geometry_msgs::msg::TransformStamped t; //Create a transform stamped object
    
    // Look up for the transformation between target_frame and fixed_frame frames
    try{
        t = tf_buffer_->lookupTransform( map_frame, robot_footprint_frame, tf2::TimePointZero);
      }catch (const tf2::TransformException & ex) {
        RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s",
            map_frame.c_str(), robot_footprint_frame.c_str(), ex.what());
        return;
      }
      
      double x = t.transform.translation.x;
      double y = t.transform.translation.y;
      double z = t.transform.translation.z;
      
      // Orientation quaternion
      tf2::Quaternion q( t.transform.rotation.x, t.transform.rotation.y,
      		t.transform.rotation.z, t.transform.rotation.w);

      // 3x3 Rotation matrix from quaternion
      tf2::Matrix3x3 m(q);

      // Roll, Pitch and Yaw from rotation matrix
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw); //Euler angles in rad
      
      //Create the pose message
      auto pose_msg = std::make_unique<geometry_msgs::msg::Pose>();
      
      pose_msg->position.x = x;
      pose_msg->position.y = y;
      pose_msg->position.z = z;
      pose_msg->orientation.x = t.transform.rotation.x;
      pose_msg->orientation.y = t.transform.rotation.y;
      pose_msg->orientation.z = t.transform.rotation.z;
      pose_msg->orientation.w = t.transform.rotation.w;

      publisher_->publish(std::move(pose_msg));
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char *argv[]) {
  auto logger = rclcpp::get_logger(node_name);
  rclcpp::init(argc, argv);

  // Obtain data from command line arguments
  if (argc == 1) {
    RCLCPP_WARN(logger, "No reference frames given (map and robot_footprint). By default, '%s' and '%s' will be used", map_frame.c_str(), robot_footprint_frame.c_str());
  }
  if (argc == 2) {
    map_frame = argv[1];
    RCLCPP_WARN(logger, "No robot frame given. So, 'map' = %s, and '%s' frame will be used", map_frame.c_str(),robot_footprint_frame.c_str());
  }
  if (argc >= 3){
    map_frame = argv[1];
    robot_footprint_frame = argv[2];
    RCLCPP_WARN(logger, "map_frame = %s, and robot_footprint = %s", map_frame.c_str(),robot_footprint_frame.c_str());
  }

  auto node = std::make_shared<AmclRobotPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

