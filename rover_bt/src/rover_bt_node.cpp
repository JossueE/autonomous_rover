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
#include "rover_bt/nodes/conditions/check_person_event.hpp"

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
#include "rover_bt/nodes/actions/set_person_profile.hpp"

#include <behaviortree_cpp/controls/switch_node.h>

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
  this->declare_parameter("rtabmap_odom_info_topic", "/rtabmap/odom_info_lite");
  this->declare_parameter("amcl_pose_topic", "/amcl_robot_pose");
  this->declare_parameter("scan_topic", "/scan");
  this->declare_parameter("pointcloud_topic", "/k4a/points2");
  // IMU is the Azure Kinect DK's onboard IMU (raw device output), not the old
  // witmotion sensor. The Kinect driver publishes /k4a/imu; imu_filter_madgwick
  // republishes it as /k4a/imu_filtered for rtabmap. We watch the raw topic so a
  // dead Kinect IMU is caught directly, independent of the filter node.
  this->declare_parameter("imu_topic", "/k4a/imu");
  // Wheel encoder feedback. Both robots' drivers (zlac706 / zlac8015d) publish
  // these std_msgs/Float64 topics; a message on either = "wheels reporting".
  this->declare_parameter("wheel_left_topic", "wheel/left_data");
  this->declare_parameter("wheel_right_topic", "wheel/right_data");
  // Lidar exists only in simulation; real hardware has none. Left false here and
  // enabled from the sim launch so the lidar watchdog never false-alarms on HW.
  this->declare_parameter("monitor_lidar", false);
  // Wheel drivers run on real hardware but not in Gazebo sim, so default true
  // and disable from the sim launch.
  this->declare_parameter("monitor_wheels", true);
  this->declare_parameter("trajectory_topic", "/sdv_trajectory");
  this->declare_parameter("command_topic", "/rover_bt/commands");
  this->declare_parameter("joy_topic", "/joy");
  this->declare_parameter("status_topic", "/rover_bt/status");
  // Person-tracker (person_tracker package) integration. The BT enables the
  // follower only in PERSON_TRACK and reads its coarse FSM state for speech.
  this->declare_parameter("person_enable_topic", "/person_tracker/enable");
  this->declare_parameter("person_profile_topic", "/person_tracker/profile");
  this->declare_parameter("person_status_topic", "/person_tracker/status");
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
  this->declare_parameter("tts_topic", "/rover_bt/tts/say");
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
    // Resolve package:// URIs: "package://pkg/relative/path" → install share path
    if (osm_map.rfind("package://", 0) == 0) {
      std::string rest = osm_map.substr(10);
      auto slash = rest.find('/');
      if (slash != std::string::npos) {
        std::string pkg = rest.substr(0, slash);
        std::string rel = rest.substr(slash + 1);
        osm_map = ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
      }
    }
    location_registry_->loadFromOsm(osm_map);
  }

  bool tts_enabled = this->get_parameter("tts_enabled").as_bool();
  if (tts_enabled) {
    std::string tts_topic = this->get_parameter("tts_topic").as_string();
    tts_ = std::make_unique<TTSClient>(this, this->get_logger(), tts_topic);
  }

  // Create SharedContext
  ctx_ = std::make_shared<SharedContext>();
  ctx_->node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}); // non-owning shared_ptr
  ctx_->arbitrator = arbitrator_.get();
  ctx_->location_registry = location_registry_.get();
  ctx_->tts = tts_.get();
  ctx_->node_start_time.store(this->get_clock()->now().seconds());
  ctx_->monitor_lidar.store(this->get_parameter("monitor_lidar").as_bool());
  ctx_->monitor_wheels.store(this->get_parameter("monitor_wheels").as_bool());

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

  // Person-tracker control. enable/profile latch (TRANSIENT_LOCAL) so the
  // tracker gets the current state even if it subscribes after us.
  auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  std::string person_enable_topic = this->get_parameter("person_enable_topic").as_string();
  person_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(person_enable_topic, latched_qos);
  ctx_->person_enable_pub = person_enable_pub_;

  std::string person_profile_topic = this->get_parameter("person_profile_topic").as_string();
  person_profile_pub_ = this->create_publisher<std_msgs::msg::String>(person_profile_topic, latched_qos);
  ctx_->person_profile_pub = person_profile_pub_;

  // Subscribers
  std::string odom_topic = this->get_parameter("odom_topic").as_string();
  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, odom_qos, std::bind(&RoverBTNode::on_odom, this, std::placeholders::_1));

  std::string rtabmap_odom_info_topic = this->get_parameter("rtabmap_odom_info_topic").as_string();
  rtabmap_odom_info_sub_ = this->create_subscription<rtabmap_msgs::msg::OdomInfo>(
    rtabmap_odom_info_topic, odom_qos,
    std::bind(&RoverBTNode::on_rtabmap_odom_info, this, std::placeholders::_1));

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

  // Wheel encoder liveness: both feedback topics share one callback that just
  // stamps last_motor_time. Float64 carries no header, so we use receive time.
  std::string wheel_left_topic = this->get_parameter("wheel_left_topic").as_string();
  wheel_left_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    wheel_left_topic, 10, std::bind(&RoverBTNode::on_wheel_data, this, std::placeholders::_1));

  std::string wheel_right_topic = this->get_parameter("wheel_right_topic").as_string();
  wheel_right_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    wheel_right_topic, 10, std::bind(&RoverBTNode::on_wheel_data, this, std::placeholders::_1));

  std::string trajectory_topic = this->get_parameter("trajectory_topic").as_string();
  trajectory_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    trajectory_topic, 10, std::bind(&RoverBTNode::on_trajectory, this, std::placeholders::_1));

  std::string command_topic = this->get_parameter("command_topic").as_string();
  command_sub_ = this->create_subscription<rover_bt::msg::Command>(
    command_topic, 10, std::bind(&RoverBTNode::on_command, this, std::placeholders::_1));

  std::string joy_topic = this->get_parameter("joy_topic").as_string();
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic, 10, std::bind(&RoverBTNode::on_joy, this, std::placeholders::_1));

  // Person-tracker coarse state (latched to match the tracker's publisher).
  std::string person_status_topic = this->get_parameter("person_status_topic").as_string();
  person_status_sub_ = this->create_subscription<std_msgs::msg::String>(
    person_status_topic, latched_qos,
    std::bind(&RoverBTNode::on_person_status, this, std::placeholders::_1));

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
  factory_.registerNodeType<CheckPersonEvent>("CheckPersonEvent");

  // BehaviorTree.CPP registers Switch2..Switch6 by default; the mode dispatch
  // needs a 7th case (PERSON_TRACK), so register Switch7 explicitly.
  factory_.registerNodeType<BT::SwitchNode<7>>("Switch7");

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
  factory_.registerNodeType<SetPersonProfile>("SetPersonProfile");

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
  monitor_lidar_        = this->get_parameter("monitor_lidar").as_bool();
  monitor_wheels_       = this->get_parameter("monitor_wheels").as_bool();

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

  // Drive the person-tracker enable from the (single source of truth) mode every
  // tick: the follower runs only in PERSON_TRACK, so any exit — including an
  // emergency forcing mode=EMERGENCY in the Safety layer — silences it within
  // one tick (100 ms), before the BT's ZeroTwist on /cmd_vel_safe goes stale.
  if (person_enable_pub_) {
    std::string mode = "IDLE";
    (void)tree_.rootBlackboard()->get("mode", mode);
    std_msgs::msg::Bool en;
    en.data = (mode == "PERSON_TRACK");
    person_enable_pub_->publish(en);
  }
}

void RoverBTNode::on_person_status(const std_msgs::msg::String::SharedPtr msg) {
  const std::string& state = msg->data;
  const std::string& prev = last_person_state_;

  // The follower just gave up (reached LOST_STOPPED): tell the BT to speak once.
  if (state == "LOST_STOPPED" && prev != "LOST_STOPPED") {
    ctx_->person_lost_event.store(true);
  }
  // Re-acquired the person after having lost it: speak the "found" line once.
  const bool was_lost = (prev == "LOST_STOPPED" || prev == "LOST_RECOVERING");
  if (state == "TRACKING" && was_lost) {
    ctx_->person_found_event.store(true);
  }

  last_person_state_ = state;
}

namespace {
// Seconds (as a double) from a ROS message header stamp.
template <typename StampT>
double to_seconds(const StampT& stamp) {
  return stamp.sec + stamp.nanosec * 1e-9;
}

// Yaw (rad) from a quaternion orientation.
double quat_to_yaw(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}

// A sensor reported as intentionally disabled (e.g. not present on this robot),
// so downstream consumers don't flag it as a missing/stale sensor.
rover_bt::msg::SensorHealth disabled_health(
    const std::string& name, const std::string& detail) {
  rover_bt::msg::SensorHealth h;
  h.name = name;
  h.status = rover_bt::msg::SensorHealth::OK;
  h.last_seen_ago = -1.0f;
  h.detail = detail;
  return h;
}

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

rover_bt::msg::SensorHealth make_rtabmap_odom_health(
    double last_seen, double now, double timeout, bool lost) {
  if (!lost) {
    return make_health("odom", last_seen, now, timeout);
  }

  rover_bt::msg::SensorHealth h;
  h.name = "odom";
  h.status = rover_bt::msg::SensorHealth::STALE;
  h.last_seen_ago = last_seen > 0.0 ? static_cast<float>(now - last_seen) : -1.0f;
  h.detail = "rtabmap odometry lost";
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
  double odom_info_time = ctx_->last_rtabmap_odom_info_time.load();
  if (odom_info_time == 0.0) {
    odom_info_time = ctx_->node_start_time.load();
  }
  msg.sensor_health.push_back(
    make_rtabmap_odom_health(
      odom_info_time, now, odom_stale_timeout_, ctx_->rtabmap_odom_lost.load()));
  if (monitor_lidar_) {
    msg.sensor_health.push_back(
      make_health("lidar",  ctx_->last_lidar_time.load(),  now, lidar_stale_timeout_));
  } else {
    // Real hardware has no lidar; report it as disabled rather than stale so
    // downstream consumers don't flag a missing sensor that isn't expected.
    msg.sensor_health.push_back(disabled_health("lidar", "disabled (sim only)"));
  }
  msg.sensor_health.push_back(
    make_health("camera", ctx_->last_camera_time.load(), now, camera_stale_timeout_));
  msg.sensor_health.push_back(
    make_health("imu",    ctx_->last_imu_time.load(),    now, imu_stale_timeout_));
  if (monitor_wheels_) {
    msg.sensor_health.push_back(
      make_health("motor",  ctx_->last_motor_time.load(),  now, motor_stale_timeout_));
  } else {
    msg.sensor_health.push_back(disabled_health("motor", "disabled (sim)"));
  }

  status_pub_->publish(msg);
}

// ─── Callbacks ────────────────────────────────────────────────────────

void RoverBTNode::on_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  ctx_->last_lidar_time.store(to_seconds(msg->header.stamp));
}

void RoverBTNode::on_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  ctx_->last_camera_time.store(to_seconds(msg->header.stamp));
}

void RoverBTNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  ctx_->last_odom_time.store(to_seconds(msg->header.stamp));

  // Extract pose info
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double yaw = quat_to_yaw(msg->pose.pose.orientation);

  // Update only if pose source is still odom
  std::lock_guard<std::mutex> lock(ctx_->pose_source_mutex);
  if (ctx_->pose_source == "odom") {
    ctx_->robot_x.store(x);
    ctx_->robot_y.store(y);
    ctx_->robot_theta.store(yaw);
  }
  ctx_->robot_linear_vel.store(msg->twist.twist.linear.x);
}

void RoverBTNode::on_rtabmap_odom_info(const rtabmap_msgs::msg::OdomInfo::SharedPtr msg) {
  const double now = this->get_clock()->now().seconds();
  ctx_->last_rtabmap_odom_info_time.store(now);
  // Stamp the moment odom *becomes* lost so the watchdog can debounce brief
  // dropouts; clear the stamp as soon as it recovers.
  const bool was_lost = ctx_->rtabmap_odom_lost.exchange(msg->lost);
  if (msg->lost && !was_lost) {
    ctx_->rtabmap_odom_lost_since.store(now);
  } else if (!msg->lost) {
    ctx_->rtabmap_odom_lost_since.store(0.0);
  }
}

void RoverBTNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  ctx_->last_imu_time.store(to_seconds(msg->header.stamp));
}

void RoverBTNode::on_wheel_data(const std_msgs::msg::Float64::SharedPtr /*msg*/) {
  // std_msgs/Float64 has no header; the fact that a message arrived is the
  // signal we need ("a wheel encoder is reporting"). The 8015d driver skips
  // publishing when it can't read the encoder, so silence => wheels not OK.
  ctx_->last_motor_time.store(this->get_clock()->now().seconds());
}

void RoverBTNode::on_amcl_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(ctx_->pose_source_mutex);
  ctx_->pose_source = "map";
  
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double yaw = quat_to_yaw(msg->pose.pose.orientation);

  ctx_->robot_x.store(x);
  ctx_->robot_y.store(y);
  ctx_->robot_theta.store(yaw);
}

void RoverBTNode::on_trajectory(const nav_msgs::msg::Path::SharedPtr msg) {
  ctx_->last_trajectory_time.store(to_seconds(msg->header.stamp));
}

void RoverBTNode::on_command(const rover_bt::msg::Command::SharedPtr msg) {
  arbitrator_->enqueue(*msg);
}

void RoverBTNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
  // Mirror teleop_joycon's three-way R1 detection: analog axis (axes[5] < 0),
  // button 7 (RT on some modes), or button 5 (R1 digital).
  const bool deadman =
    (msg->axes.size()    > 5 && msg->axes[5]   < 0.0f) ||
    (msg->buttons.size() > 7 && msg->buttons[7] == 1)  ||
    (msg->buttons.size() > 5 && msg->buttons[5] == 1);
  if (deadman) {
    ctx_->last_joy_active_time.store(this->now().seconds());
  }

  rover_bt::msg::Command cmd;
  cmd.stamp = this->now();
  cmd.source = "joycon";
  cmd.priority = 1;

  // Button 0 (A / Cross) → stop; Button 3 (Y / Triangle) → switch to autonomous.
  if (msg->buttons.size() > 0 && msg->buttons[0] == 1) {
    cmd.command = "stop";
    arbitrator_->enqueue(cmd);
  } else if (msg->buttons.size() > 3 && msg->buttons[3] == 1) {
    cmd.command = "autonomous";
    arbitrator_->enqueue(cmd);
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
