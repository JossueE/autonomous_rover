#include "rover_bt/rover_bt_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Include ALL custom BT nodes
#include "rover_bt/nodes/conditions/check_mode.hpp"
#include "rover_bt/nodes/conditions/check_command.hpp"
#include "rover_bt/nodes/conditions/check_sensor_health.hpp"
#include "rover_bt/nodes/conditions/check_nav_status.hpp"
#include "rover_bt/nodes/conditions/check_flag.hpp"
#include "rover_bt/nodes/conditions/check_joy_active.hpp"

#include "rover_bt/nodes/actions/navigate_to_goal.hpp"
#include "rover_bt/nodes/actions/zero_twist.hpp"
#include "rover_bt/nodes/actions/move_rover.hpp"
#include "rover_bt/nodes/actions/speak.hpp"
#include "rover_bt/nodes/actions/set_mode.hpp"
#include "rover_bt/nodes/actions/set_flag.hpp"
#include "rover_bt/nodes/actions/clear_command.hpp"
#include "rover_bt/nodes/actions/start_mapping.hpp"
#include "rover_bt/nodes/actions/stop_mapping.hpp"
#include "rover_bt/nodes/actions/request_replan.hpp"
#include "rover_bt/nodes/actions/publish_status.hpp"
#include "rover_bt/nodes/actions/increment_patrol_index.hpp"
#include "rover_bt/nodes/actions/save_location.hpp"
#include "rover_bt/nodes/actions/process_command.hpp"

namespace rover_bt {

RoverBTNode::RoverBTNode() : Node("rover_bt_node") {
  init_parameters();
  init_subsystems();
  init_ros_interfaces();
  init_behavior_tree();
  RCLCPP_INFO(this->get_logger(), "rover_bt C++ node initialized and running.");
}

void RoverBTNode::init_parameters() {
  this->declare_parameter("bt_tick_rate", 10.0);
  this->declare_parameter("tree_xml", "");
  this->declare_parameter("cmd_vel_topic", "/cmd_vel_safe");
  this->declare_parameter("odom_topic", "/odom");
  this->declare_parameter("amcl_pose_topic", "/amcl_robot_pose");
  this->declare_parameter("scan_topic", "/scan");
  this->declare_parameter("pointcloud_topic", "/k4a/points2");
  this->declare_parameter("imu_topic", "/imu/data");
  this->declare_parameter("trajectory_topic", "/sdv_trajectory");
  this->declare_parameter("command_topic", "/rover_bt/commands");
  this->declare_parameter("joy_topic", "/joy");
  this->declare_parameter("status_topic", "/rover_bt/status");
  this->declare_parameter("amcl_pose_timeout", 2.0);
  this->declare_parameter("goal_timeout", 30.0);
  this->declare_parameter("planner_heartbeat_timeout", 10.0);
  this->declare_parameter("nmpc_stall_velocity", 0.02);
  this->declare_parameter("nmpc_stall_timeout", 15.0);
  this->declare_parameter("goal_tolerance", 0.3);
  this->declare_parameter("lidar_stale_timeout", 5.0);
  this->declare_parameter("camera_stale_timeout", 5.0);
  this->declare_parameter("odom_stale_timeout", 3.0);
  this->declare_parameter("imu_stale_timeout", 5.0);
  this->declare_parameter("motor_stale_timeout", 5.0);
  this->declare_parameter("tts_enabled", true);
  this->declare_parameter("piper_bin", "");
  this->declare_parameter("piper_model", "");
  this->declare_parameter("lanelet2_map", "");
  this->declare_parameter("dynamic_waypoints_file", "");
  // patrol_waypoints: declared with dynamic typing so it tolerates being set
  // (or left empty) from YAML/launch. NOTE: an *empty* YAML list
  // (patrol_waypoints: []) loads as PARAMETER_NOT_SET and aborts the node, so
  // the shared param file leaves it unset rather than [] — see rover_bt_params.yaml.
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;
    this->declare_parameter("patrol_waypoints",
                            rclcpp::ParameterValue(std::vector<std::string>()), desc);
  }
}

void RoverBTNode::init_subsystems() {
  arbitrator_ = std::make_unique<CommandArbitrator>();
  
  std::string waypoints_file = this->get_parameter("dynamic_waypoints_file").as_string();
  if (waypoints_file.empty()) {
    std::string share_dir = ament_index_cpp::get_package_share_directory("rover_bt");
    waypoints_file = share_dir + "/config/waypoints.yaml";
  }
  
  location_registry_ = std::make_unique<LocationRegistry>();
  location_registry_->loadFromYaml(waypoints_file);

  std::string osm_map = this->get_parameter("lanelet2_map").as_string();
  if (!osm_map.empty()) {
    location_registry_->loadFromOsm(osm_map);
  }

  std::string piper_bin = this->get_parameter("piper_bin").as_string();
  std::string piper_model = this->get_parameter("piper_model").as_string();
  bool tts_enabled = this->get_parameter("tts_enabled").as_bool();

  if (tts_enabled) {
    tts_ = std::make_unique<TTSClient>(this->get_logger(), piper_bin, piper_model);
  }

  // Create SharedContext
  ctx_ = std::make_shared<SharedContext>();
  ctx_->node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}); // non-owning shared_ptr
  ctx_->arbitrator = arbitrator_.get();
  ctx_->location_registry = location_registry_.get();
  ctx_->tts = tts_.get();

  // Load patrol waypoints from parameters.
  // An empty YAML list (patrol_waypoints: []) is loaded as PARAMETER_NOT_SET,
  // so as_string_array() would throw. Guard against that and treat it as empty.
  std::vector<std::string> patrol_wps;
  {
    const auto& p = this->get_parameter("patrol_waypoints");
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      patrol_wps = p.as_string_array();
    } else if (p.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_WARN(this->get_logger(),
                  "patrol_waypoints has unexpected type; ignoring (treating as empty).");
    }
  }
  ctx_->patrol_waypoints = patrol_wps;
}

void RoverBTNode::init_ros_interfaces() {
  // Publishers
  std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
  ctx_->cmd_vel_pub = cmd_vel_pub_;

  std::string status_topic = this->get_parameter("status_topic").as_string();
  status_pub_ = this->create_publisher<rover_bt::msg::RoverStatus>(status_topic, 10);

  // Subscribers
  std::string odom_topic = this->get_parameter("odom_topic").as_string();
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&RoverBTNode::on_odom, this, std::placeholders::_1));

  std::string amcl_pose_topic = this->get_parameter("amcl_pose_topic").as_string();
  amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    amcl_pose_topic, 10, std::bind(&RoverBTNode::on_amcl_pose, this, std::placeholders::_1));

  std::string scan_topic = this->get_parameter("scan_topic").as_string();
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, 10, std::bind(&RoverBTNode::on_laser_scan, this, std::placeholders::_1));

  std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
  camera_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic, 10, std::bind(&RoverBTNode::on_point_cloud, this, std::placeholders::_1));

  std::string imu_topic = this->get_parameter("imu_topic").as_string();
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, 10, std::bind(&RoverBTNode::on_imu, this, std::placeholders::_1));

  std::string trajectory_topic = this->get_parameter("trajectory_topic").as_string();
  trajectory_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    trajectory_topic, 10, std::bind(&RoverBTNode::on_trajectory, this, std::placeholders::_1));

  std::string command_topic = this->get_parameter("command_topic").as_string();
  command_sub_ = this->create_subscription<rover_bt::msg::Command>(
    command_topic, 10, std::bind(&RoverBTNode::on_command, this, std::placeholders::_1));

  std::string joy_topic = this->get_parameter("joy_topic").as_string();
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic, 10, std::bind(&RoverBTNode::on_joy, this, std::placeholders::_1));

  // Services
  send_command_srv_ = this->create_service<rover_bt::srv::SendCommand>(
    "/rover_bt/send_command", std::bind(&RoverBTNode::handle_send_command, this, std::placeholders::_1, std::placeholders::_2));

  save_location_srv_ = this->create_service<rover_bt::srv::SaveLocation>(
    "/rover_bt/save_location", std::bind(&RoverBTNode::handle_save_location, this, std::placeholders::_1, std::placeholders::_2));
}

void RoverBTNode::init_behavior_tree() {
  // Register conditions
  factory_.registerNodeType<CheckMode>("CheckMode");
  factory_.registerNodeType<CheckCommand>("CheckCommand");
  factory_.registerNodeType<CheckSensorHealth>("CheckSensorHealth");
  factory_.registerNodeType<CheckNavStatus>("CheckNavStatus");
  factory_.registerNodeType<CheckFlag>("CheckFlag");
  factory_.registerNodeType<CheckJoyActive>("CheckJoyActive");

  // Register actions
  factory_.registerNodeType<NavigateToGoal>("NavigateToGoal");
  factory_.registerNodeType<ZeroTwist>("ZeroTwist");
  factory_.registerNodeType<MoveRover>("MoveRover");
  factory_.registerNodeType<Speak>("Speak");
  factory_.registerNodeType<SetMode>("SetMode");
  factory_.registerNodeType<SetFlag>("SetFlag");
  factory_.registerNodeType<ClearCommand>("ClearCommand");
  factory_.registerNodeType<StartMapping>("StartMapping");
  factory_.registerNodeType<StopMapping>("StopMapping");
  factory_.registerNodeType<RequestReplan>("RequestReplan");
  factory_.registerNodeType<PublishStatus>("PublishStatus");
  factory_.registerNodeType<IncrementPatrolIndex>("IncrementPatrolIndex");
  factory_.registerNodeType<SaveLocation>("SaveLocation");
  factory_.registerNodeType<ProcessCommand>("ProcessCommand");

  // Load XML
  std::string xml_path = this->get_parameter("tree_xml").as_string();
  if (xml_path.empty()) {
    std::string share_dir = ament_index_cpp::get_package_share_directory("rover_bt");
    xml_path = share_dir + "/trees/rover_bt_main.xml";
  }

  RCLCPP_INFO(this->get_logger(), "Loading BT XML from: %s", xml_path.c_str());
  tree_ = factory_.createTreeFromFile(xml_path);

  // Initialize Blackboard keys
  auto blackboard = tree_.rootBlackboard();
  blackboard->set("context", ctx_);
  blackboard->set("mode", std::string("IDLE"));
  blackboard->set("command", std::string(""));
  blackboard->set("target_location", std::string(""));
  blackboard->set("nav_status", std::string("idle"));
  blackboard->set("is_mapping", false);
  blackboard->set("mapping_mode", std::string("off"));
  blackboard->set("patrol_index", -1);
  blackboard->set("patrol_waypoint", std::string(""));
  blackboard->set("recovery_attempted", false);
  blackboard->set("active_command_source", std::string(""));
  blackboard->set("goal_distance", -1.0);

  // Cache stale timeouts for status reporting
  lidar_stale_timeout_  = this->get_parameter("lidar_stale_timeout").as_double();
  camera_stale_timeout_ = this->get_parameter("camera_stale_timeout").as_double();
  odom_stale_timeout_   = this->get_parameter("odom_stale_timeout").as_double();
  imu_stale_timeout_    = this->get_parameter("imu_stale_timeout").as_double();
  motor_stale_timeout_  = this->get_parameter("motor_stale_timeout").as_double();

  // Tick timer (10Hz)
  double tick_rate = this->get_parameter("bt_tick_rate").as_double();
  auto period = std::chrono::duration<double>(1.0 / tick_rate);
  tick_timer_ = this->create_wall_timer(period, std::bind(&RoverBTNode::tick_tree, this));

  // Status publication timer (~1 Hz)
  status_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), std::bind(&RoverBTNode::publish_status, this));
}

void RoverBTNode::tick_tree() {
  // Tick tree exactly once
  tree_.tickExactlyOnce();
}

namespace {
// Build a SensorHealth entry from the last-seen timestamp.
rover_bt::msg::SensorHealth make_health(
    const std::string& name, double last_seen, double now, double timeout) {
  rover_bt::msg::SensorHealth h;
  h.name = name;
  if (last_seen <= 0.0) {
    // No data ever received (startup grace period).
    h.status = rover_bt::msg::SensorHealth::STALE;
    h.last_seen_ago = -1.0f;
    h.detail = "no data yet";
    return h;
  }
  double ago = now - last_seen;
  h.last_seen_ago = static_cast<float>(ago);
  if (ago <= timeout) {
    h.status = rover_bt::msg::SensorHealth::OK;
    h.detail = "ok";
  } else {
    h.status = rover_bt::msg::SensorHealth::STALE;
    h.detail = "stale";
  }
  return h;
}
}  // namespace

void RoverBTNode::publish_status() {
  if (!tree_.rootNode()) {
    return;
  }
  auto bb = tree_.rootBlackboard();

  rover_bt::msg::RoverStatus msg;
  msg.stamp = this->now();

  std::string mode = "IDLE";
  (void)bb->get("mode", mode);
  msg.mode = mode;

  std::string source = "";
  (void)bb->get("active_command_source", source);
  msg.active_command_source = source;

  std::string nav_status = "idle";
  (void)bb->get("nav_status", nav_status);
  msg.navigation_status = nav_status;

  double goal_distance = -1.0;
  (void)bb->get("goal_distance", goal_distance);
  msg.goal_distance = static_cast<float>(goal_distance);

  std::string target = "";
  (void)bb->get("target_location", target);
  msg.target_location = target;

  bool is_mapping = false;
  (void)bb->get("is_mapping", is_mapping);
  msg.is_mapping = is_mapping;

  std::string mapping_mode = "off";
  (void)bb->get("mapping_mode", mapping_mode);
  msg.mapping_mode = mapping_mode;

  // Sensor health, using the same clock the watchdogs use.
  double now = this->get_clock()->now().seconds();
  msg.sensor_health.push_back(
    make_health("odom",   ctx_->last_odom_time.load(),   now, odom_stale_timeout_));
  msg.sensor_health.push_back(
    make_health("lidar",  ctx_->last_lidar_time.load(),  now, lidar_stale_timeout_));
  msg.sensor_health.push_back(
    make_health("camera", ctx_->last_camera_time.load(), now, camera_stale_timeout_));
  msg.sensor_health.push_back(
    make_health("imu",    ctx_->last_imu_time.load(),    now, imu_stale_timeout_));
  msg.sensor_health.push_back(
    make_health("motor",  ctx_->last_motor_time.load(),  now, motor_stale_timeout_));

  status_pub_->publish(msg);
}

// ─── Callbacks ────────────────────────────────────────────────────────

void RoverBTNode::on_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  ctx_->last_lidar_time.store(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
}

void RoverBTNode::on_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  ctx_->last_camera_time.store(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
}

void RoverBTNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  ctx_->last_odom_time.store(t);

  // Extract pose info
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Update only if pose source is still odom
  std::lock_guard<std::mutex> lock(ctx_->pose_source_mutex);
  if (ctx_->pose_source == "odom") {
    ctx_->robot_x.store(x);
    ctx_->robot_y.store(y);
    ctx_->robot_theta.store(yaw);
  }
  ctx_->robot_linear_vel.store(msg->twist.twist.linear.x);
}

void RoverBTNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  ctx_->last_imu_time.store(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
}

void RoverBTNode::on_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(ctx_->pose_source_mutex);
  ctx_->pose_source = "map";
  
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ctx_->robot_x.store(x);
  ctx_->robot_y.store(y);
  ctx_->robot_theta.store(yaw);
}

void RoverBTNode::on_trajectory(const nav_msgs::msg::Path::SharedPtr msg) {
  ctx_->last_trajectory_time.store(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
}

void RoverBTNode::on_command(const rover_bt::msg::Command::SharedPtr msg) {
  arbitrator_->enqueue(*msg);
}

void RoverBTNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
  // button 4 (L1) or 5 (R1) as deadman switch — nothing happens unless held.
  const bool deadman = msg->buttons.size() > 5 &&
                       (msg->buttons[4] == 1 || msg->buttons[5] == 1);
  if (!deadman) {
    return;
  }

  // Mark the joystick as active. The behavior tree's JoyconOverlay uses this
  // (via CheckJoyActive) to switch into TELEOP_JOYCON automatically while the
  // deadman is held, and back to IDLE shortly after it is released — joycon
  // teleop is detected, never a voice command.
  ctx_->last_joy_active_time.store(this->now().seconds());

  rover_bt::msg::Command cmd;
  cmd.stamp = this->now();
  cmd.source = "joycon";
  cmd.priority = 1;  // Priority 1 for joycon commands.

  // Stop command mapped to button 0 (A or Cross)
  if (msg->buttons.size() > 0 && msg->buttons[0] == 1) {
    cmd.command = "stop";
    arbitrator_->enqueue(cmd);
  } else if (msg->buttons.size() > 3 && msg->buttons[3] == 1) { // Button Y or Triangle
    cmd.command = "autonomous";
    arbitrator_->enqueue(cmd);
  } else if (msg->axes.size() > 3 &&
             (std::abs(msg->axes[1]) > 0.2 || std::abs(msg->axes[3]) > 0.2)) {
    // Publishes direct cmd_vel velocities while the stick is deflected.
    geometry_msgs::msg::Twist twist;
    twist.linear.x = msg->axes[1] * 0.4;  // Scale velocities
    twist.angular.z = msg->axes[3] * 0.6;
    cmd_vel_pub_->publish(twist);
  }
}

// ─── Service Handlers ──────────────────────────────────────────────────

void RoverBTNode::handle_send_command(
  const std::shared_ptr<rover_bt::srv::SendCommand::Request> request,
  std::shared_ptr<rover_bt::srv::SendCommand::Response> response) {
  
  rover_bt::msg::Command cmd;
  cmd.stamp = this->now();
  cmd.source = request->source;
  cmd.command = request->command;
  cmd.target = request->target;

  // Decide priority based on command name or source
  if (request->command == "emergency_stop") {
    cmd.priority = 0;
  } else if (request->source == "joycon") {
    cmd.priority = 1;
  } else if (request->source == "voice") {
    cmd.priority = 2;
  } else if (request->source == "gui") {
    cmd.priority = 3;
  } else {
    cmd.priority = 4; // lowest (general service/external script)
  }

  arbitrator_->enqueue(cmd);
  response->accepted = true;
  response->message = "Command enqueued in priority arbitrator.";
}

void RoverBTNode::handle_save_location(
  const std::shared_ptr<rover_bt::srv::SaveLocation::Request> request,
  std::shared_ptr<rover_bt::srv::SaveLocation::Response> response) {
  
  double x = ctx_->robot_x.load();
  double y = ctx_->robot_y.load();
  double theta = ctx_->robot_theta.load();

  bool success = location_registry_->save(request->name, x, y, theta, request->lanelet);
  response->success = success;
  response->x = x;
  response->y = y;
  response->theta = theta;

  if (success) {
    response->message = "Successfully saved waypoint location: " + request->name;
  } else {
    response->message = "Failed to save waypoint location to persistent file.";
  }
}

}  // namespace rover_bt

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rover_bt::RoverBTNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
