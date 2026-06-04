#include <path_planning.hpp>

#include <chrono>
#include <cctype>
#include <functional>

namespace {

double selectConfiguredDouble(const double preferred, const double legacy) {
    return preferred > 0.0 ? preferred : legacy;
}

int selectConfiguredInt(const int preferred, const int legacy) {
    return preferred > 0 ? preferred : legacy;
}

std::string normalizeModelName(std::string model_name) {
    std::transform(model_name.begin(), model_name.end(), model_name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return model_name;
}

} // namespace

path_planning::path_planning() : Node("path_planning"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{
    this->declare_parameter<std::string>("kinematics.model", "ackermann");
    this->declare_parameter<double>("vehicle.axle_to_front", 0.0);
    this->declare_parameter<double>("vehicle.axle_to_back", 0.0);
    this->declare_parameter<double>("vehicle.width", 0.0);
    this->declare_parameter<int>("planner.segment_steps", 0);
    this->declare_parameter<double>("planner.sample_distance", 0.0);
    this->declare_parameter<double>("planner.square_size_m", 1.6);
    this->declare_parameter<double>("planner.safe_clear", 0.2);
    this->declare_parameter<int>("planner.obstacle_inflation_radius_cells", 1);
    this->declare_parameter<double>("planner.obstacle_inflation_radius_m", 0.0);
    this->declare_parameter<int>("planner.branching_factor", 0);
    this->declare_parameter<double>("planner.forward_distance", forward_distance);
    this->declare_parameter<int>("planner.chunk_radius_cells", chunk_radius);
    this->declare_parameter<double>("planner.scale_factor", scale_factor);
    this->declare_parameter<double>("ackermann.wheelbase", 0.0);
    this->declare_parameter<double>("ackermann.max_steering_angle", 0.0);
    this->declare_parameter<double>("differential.linear_step", 0.0);
    this->declare_parameter<double>("differential.max_angular_step", 0.2);
    this->declare_parameter<int>("differential.angular_samples", 0);
    this->declare_parameter<bool>("differential.include_in_place_rotation", true);

    this->declare_parameter<double>("maxSteerAngle", 0.0);
    this->declare_parameter<double>("wheelBase", 0.0);
    this->declare_parameter<double>("axleToFront", 0.0);
    this->declare_parameter<double>("axleToBack", 0.0);
    this->declare_parameter<double>("width", 0.0);
    this->declare_parameter<int>("pathLength", 0);
    this->declare_parameter<double>("step_car", 0.0);
    this->declare_parameter<int>("tree_depth", 3);
    this->declare_parameter<int>("branching_factor", 5);
    this->declare_parameter<std::string>("map_path", "");
    this->declare_parameter<double>("x_offset", 0.0);
    this->declare_parameter<double>("y_offset", 0.0);
    this->declare_parameter<double>("z_offset", 0.0);
    this->declare_parameter<std::string>("pose_frame", "lidar_link");
    this->declare_parameter<int>("start_lanelet_id", 0);
    this->declare_parameter<int>("end_lanelet_id", 0);
    this->declare_parameter<std::string>("start_lanelet_name", "");
    this->declare_parameter<std::string>("end_lanelet_name", "");

    // Occupancy grid parameters
    this->declare_parameter<double>("global_planner_resolution", 0.20);
    this->declare_parameter<int>("global_planner_close_radius", 0);
    this->declare_parameter<int>("global_planner_close_iters", 0);
    this->declare_parameter<int>("global_planner_outside_value", 100);
    this->declare_parameter<std::string>("global_planner_frame_id", "map");
    this->declare_parameter<std::string>("global_planner_occupancy_output_topic", "occupancy_grid_complete_map");

    std::string configured_model;
    double vehicle_axle_to_front = 0.0;
    double vehicle_axle_to_back = 0.0;
    double vehicle_width = 0.0;
    int configured_segment_steps = 0;
    double configured_sample_distance = 0.0;
    double configured_square_size_m = 1.6;
    double configured_safe_clear = 0.2;
    int configured_obstacle_inflation_radius_cells = 1;
    int configured_branching_factor = 0;
    double configured_ackermann_wheelbase = 0.0;
    double configured_ackermann_max_steering_angle = 0.0;

    this->get_parameter("kinematics.model", configured_model);
    this->get_parameter("vehicle.axle_to_front", vehicle_axle_to_front);
    this->get_parameter("vehicle.axle_to_back", vehicle_axle_to_back);
    this->get_parameter("vehicle.width", vehicle_width);
    this->get_parameter("planner.segment_steps", configured_segment_steps);
    this->get_parameter("planner.sample_distance", configured_sample_distance);
    this->get_parameter("planner.square_size_m", configured_square_size_m);
    this->get_parameter("planner.safe_clear", configured_safe_clear);
    this->get_parameter("planner.obstacle_inflation_radius_cells", configured_obstacle_inflation_radius_cells);
    this->get_parameter("planner.obstacle_inflation_radius_m", obstacle_inflation_radius_m_);
    this->get_parameter("planner.branching_factor", configured_branching_factor);
    this->get_parameter("planner.forward_distance", forward_distance);
    this->get_parameter("planner.chunk_radius_cells", chunk_radius);
    this->get_parameter("planner.scale_factor", scale_factor);
    this->get_parameter("ackermann.wheelbase", configured_ackermann_wheelbase);
    this->get_parameter("ackermann.max_steering_angle", configured_ackermann_max_steering_angle);
    this->get_parameter("differential.linear_step", differential_linear_step_);
    this->get_parameter("differential.max_angular_step", differential_max_angular_step_);
    this->get_parameter("differential.angular_samples", differential_angular_samples_);
    this->get_parameter("differential.include_in_place_rotation", differential_include_in_place_rotation_);

    double legacy_max_steer_angle = 0.0;
    double legacy_wheelbase = 0.0;
    double legacy_axle_to_front = 0.0;
    double legacy_axle_to_back = 0.0;
    double legacy_width = 0.0;
    int legacy_path_length = 0;
    double legacy_step_car = 0.0;
    int legacy_branching_factor = 0;

    this->get_parameter("maxSteerAngle", legacy_max_steer_angle);
    this->get_parameter("wheelBase", legacy_wheelbase);
    this->get_parameter("axleToFront", legacy_axle_to_front);
    this->get_parameter("axleToBack", legacy_axle_to_back);
    this->get_parameter("width", legacy_width);
    this->get_parameter("pathLength", legacy_path_length);
    this->get_parameter("step_car", legacy_step_car);
    this->get_parameter("tree_depth", tree_depth);
    this->get_parameter("branching_factor", legacy_branching_factor);
    this->get_parameter("map_path", map_path_);
    this->get_parameter("x_offset", x_offset_);
    this->get_parameter("y_offset", y_offset_);
    this->get_parameter("z_offset", z_offset_);
    this->get_parameter("pose_frame", pose_frame_);
    this->get_parameter("start_lanelet_id", start_lanelet_id_);
    this->get_parameter("end_lanelet_id", end_lanelet_id_);
    this->get_parameter("start_lanelet_name", start_lanelet_name_);
    this->get_parameter("end_lanelet_name", end_lanelet_name_);

    kinematic_model_name_ = normalizeModelName(configured_model);
    axle_to_front_ = selectConfiguredDouble(vehicle_axle_to_front, legacy_axle_to_front);
    axle_to_back_ = selectConfiguredDouble(vehicle_axle_to_back, legacy_axle_to_back);
    vehicle_width_ = selectConfiguredDouble(vehicle_width, legacy_width);
    pathLength = selectConfiguredInt(configured_segment_steps, legacy_path_length);
    step_car = selectConfiguredDouble(configured_sample_distance, legacy_step_car);
    square_size_m_ = configured_square_size_m > 0.0 ? configured_square_size_m : 1.6;
    SAFE_CLEAR = configured_safe_clear > 0.0 ? configured_safe_clear : 0.2;
    obstacle_inflation_radius_cells_ =
        std::max(0, configured_obstacle_inflation_radius_cells);
    obstacle_inflation_radius_m_ = std::max(0.0, obstacle_inflation_radius_m_);

    // chunk_radius is parameter-driven; keep chunk_size in sync (cells == 2*radius)
    chunk_radius = std::max(1, chunk_radius);
    chunk_size = chunk_radius * 2;
    if (scale_factor <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "planner.scale_factor must be > 0 (got %f); falling back to 1.0.",
            scale_factor);
        scale_factor = 1.0;
    }
    branching_factor = selectConfiguredInt(configured_branching_factor, legacy_branching_factor);
    ackermann_wheelbase_ = selectConfiguredDouble(configured_ackermann_wheelbase, legacy_wheelbase);
    ackermann_max_steering_angle_ =
        selectConfiguredDouble(configured_ackermann_max_steering_angle, legacy_max_steer_angle);
    if (differential_linear_step_ <= 0.0) {
        differential_linear_step_ = step_car;
    }
    if (differential_angular_samples_ <= 0) {
        differential_angular_samples_ = branching_factor;
    }

    // Occupancy grid parameters
    this->get_parameter("global_planner_resolution", global_planner_resolution_);
    this->get_parameter("global_planner_close_radius", global_planner_close_radius_);
    this->get_parameter("global_planner_close_iters", global_planner_close_iters_);
    this->get_parameter("global_planner_outside_value", global_planner_outside_value_);
    this->get_parameter("global_planner_frame_id", global_planner_frame_id_);
    this->get_parameter("global_planner_occupancy_output_topic", global_planner_occupancy_output_topic_);

    obstacle_info_subscription_ = this->create_subscription<path_planning_dynamic::msg::ObstacleCollection>(
        "/obstacle_info", 10, std::bind(&path_planning::obstacle_info_callback, this, std::placeholders::_1));

    // publisher for the occupancy grid of the obstacles
    occupancy_grid_pub_test_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid_obstacles", 10);
    
    car_analytics_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/car", 10);
    
    real_trajectories_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/none_real_traj", 10);

    real_trajectories_pub_2 = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/real_trajectories_option_2", 10);

    all_paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/all_available_paths", 10);

    sdv_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/sdv_trajectory", 10);

    global_planner_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/global_planner", 10);

    global_planner_occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        global_planner_occupancy_output_topic_, 10);

    // -------------> Initialize the shared pointers  <------------
    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    rescaled_chunk_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    car_state_ = std::make_shared<State>();
    grid_map_ = nullptr;

    vehicle_footprint_.setDimensions(axle_to_front_, axle_to_back_, vehicle_width_);
    AckermannKinematicsConfig ackermann_config;
    ackermann_config.wheelbase = ackermann_wheelbase_;
    ackermann_config.max_steering_angle = ackermann_max_steering_angle_;
    ackermann_config.linear_step = step_car;

    DifferentialKinematicsConfig differential_config;
    differential_config.linear_step = differential_linear_step_;
    differential_config.max_angular_step = differential_max_angular_step_;
    differential_config.moving_angular_samples = differential_angular_samples_;
    differential_config.include_in_place_rotation = differential_include_in_place_rotation_;

    kinematic_model_ =
        makeKinematicModel(kinematic_model_name_, ackermann_config, differential_config);
    motion_primitives_ = kinematic_model_->buildMotionPrimitives(branching_factor);

    auto markers = vehicle_footprint_.toMarkerArray("base_footprint", this->get_clock()->now());

    car_analytics_->publish(markers);


    // log out parameters
    RCLCPP_INFO(this->get_logger(), "\033[1;34mkinematics.model: %s\033[0m", kinematic_model_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "\033[1;34mackermann.max_steering_angle: %f\033[0m", ackermann_max_steering_angle_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mackermann.wheelbase: %f\033[0m", ackermann_wheelbase_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mvehicle.axle_to_front: %f\033[0m", axle_to_front_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mvehicle.axle_to_back: %f\033[0m", axle_to_back_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mvehicle.width: %f\033[0m", vehicle_width_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mdifferential.linear_step: %f\033[0m", differential_linear_step_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mdifferential.max_angular_step: %f\033[0m", differential_max_angular_step_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mdifferential.angular_samples: %d\033[0m", differential_angular_samples_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mpathLength: %d\033[0m", pathLength);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mstep_car: %f\033[0m", step_car);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mplanner.square_size_m: %f\033[0m", square_size_m_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mplanner.safe_clear: %f\033[0m", SAFE_CLEAR);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mplanner.obstacle_inflation_radius_cells: %d\033[0m",
                obstacle_inflation_radius_cells_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mplanner.obstacle_inflation_radius_m: %f\033[0m",
                obstacle_inflation_radius_m_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mplanner.forward_distance: %f\033[0m", forward_distance);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mplanner.chunk_radius_cells: %d\033[0m", chunk_radius);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mplanner.scale_factor: %f\033[0m", scale_factor);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mtree_depth: %d\033[0m", tree_depth);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mbranching_factor: %d\033[0m", branching_factor);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mmap_path: %s\033[0m", map_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "\033[1;34mx_offset: %f\033[0m", x_offset_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34my_offset: %f\033[0m", y_offset_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mz_offset: %f\033[0m", z_offset_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mpose_frame: %s\033[0m", pose_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "\033[1;34mstart_lanelet_id: %d\033[0m", start_lanelet_id_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mend_lanelet_id: %d\033[0m", end_lanelet_id_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mstart_lanelet_name: %s\033[0m", start_lanelet_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "\033[1;34mend_lanelet_name: %s\033[0m", end_lanelet_name_.c_str());

    rebuildGlobalPlanner();
    params_handler_ = this->add_on_set_parameters_callback(
        std::bind(&path_planning::onPlannerParameters, this, std::placeholders::_1));

    // Initialize Action Server
    action_server_ = rclcpp_action::create_server<NavigateToGoal>(
        this,
        "navigate_to_goal",
        std::bind(&path_planning::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&path_planning::handle_cancel, this, std::placeholders::_1),
        std::bind(&path_planning::handle_accepted, this, std::placeholders::_1));
}


path_planning::~path_planning()
{
}

void path_planning::rebuildGlobalPlanner()
{
    global_planner_ = std::make_shared<GlobalPlanner>(
        x_offset_, y_offset_, map_path_, start_lanelet_id_, end_lanelet_id_,
        start_lanelet_name_, end_lanelet_name_, global_planner_resolution_,
        global_planner_close_radius_, global_planner_close_iters_,
        global_planner_outside_value_, global_planner_frame_id_);

    all_waypoints_from_global_planner_ = global_planner_->getAllAllWaypointsStruct();
    publishGlobalPlanner();
    if (global_planner_->isOccupancyGridReady())
    {
        global_planner_occupancy_grid_ = global_planner_->getOccupancyGrid();
        publishGlobalPlannerOccupancyGrid();
        global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(global_planner_occupancy_grid_);
    }
}

rcl_interfaces::msg::SetParametersResult path_planning::onPlannerParameters(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "accepted";

    int new_start_id = start_lanelet_id_;
    int new_end_id = end_lanelet_id_;
    std::string new_start_name = start_lanelet_name_;
    std::string new_end_name = end_lanelet_name_;
    bool should_rebuild = false;

    for (const auto &param : params) {
        const auto &name = param.get_name();
        if (name == "start_lanelet_id" || name == "end_lanelet_id") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                result.successful = false;
                result.reason = name + " must be an integer";
                return result;
            }
            if (name == "start_lanelet_id") {
                new_start_id = param.as_int();
            } else {
                new_end_id = param.as_int();
            }
            should_rebuild = true;
        } else if (name == "start_lanelet_name" || name == "end_lanelet_name") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                result.successful = false;
                result.reason = name + " must be a string";
                return result;
            }
            if (name == "start_lanelet_name") {
                new_start_name = param.as_string();
            } else {
                new_end_name = param.as_string();
            }
            should_rebuild = true;
        }
    }

    if (should_rebuild) {
        start_lanelet_id_ = new_start_id;
        end_lanelet_id_ = new_end_id;
        start_lanelet_name_ = new_start_name;
        end_lanelet_name_ = new_end_name;
        RCLCPP_INFO(this->get_logger(),
            "Rebuilding global planner: start_id=%d end_id=%d start_name='%s' end_name='%s'",
            start_lanelet_id_, end_lanelet_id_,
            start_lanelet_name_.c_str(), end_lanelet_name_.c_str());
        rebuildGlobalPlanner();
    }

    return result;
}

// =============================
// get the state of the car
// =============================
void path_planning::getCurrentRobotState()
{
    geometry_msgs::msg::Transform pose_tf;
    try
    {
        pose_tf = tf2_buffer.lookupTransform("map", pose_frame_, tf2::TimePointZero).transform;
        tf2::Quaternion quat;
        tf2::fromMsg(pose_tf.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        {
            std::lock_guard<std::mutex> lk(car_state_mutex_);
            car_state_->x = pose_tf.translation.x;
            car_state_->y = pose_tf.translation.y;
            car_state_->z = pose_tf.translation.z + z_offset_;
            car_state_->heading = yaw;
            car_state_valid_ = true;
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Transform error (map <- %s): %s", pose_frame_.c_str(), ex.what());
        std::lock_guard<std::mutex> lk(car_state_mutex_);
        car_state_valid_ = false;
    }
}

// =============================
// publish the global planner
// =============================
void path_planning::publishGlobalPlanner()
{
    RCLCPP_DEBUG(this->get_logger(), "Publishing global planner with %zu waypoints",
                 all_waypoints_from_global_planner_.size());
    global_planner_markers_.markers.clear();

    // Clear previous text markers
    visualization_msgs::msg::Marker clear_text;
    clear_text.header.frame_id = "map";
    clear_text.header.stamp = this->now();
    clear_text.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_text.ns = "global_planner_text";
    global_planner_markers_.markers.push_back(clear_text);

    // Palette for the per-lane-sequence color cycling.
    // Indices 0..9 match the previous hardcoded switch and any sequence id is
    // mapped via (seq_id % palette_size) to keep the rendering identical.
    static constexpr struct { float r, g, b; } palette[] = {
        {0.0f, 0.0f, 1.0f},   // 0: Blue (main path)
        {0.0f, 1.0f, 0.0f},   // 1: Green
        {1.0f, 0.0f, 0.0f},   // 2: Red
        {1.0f, 1.0f, 0.0f},   // 3: Yellow
        {1.0f, 0.0f, 1.0f},   // 4: Magenta
        {0.0f, 1.0f, 1.0f},   // 5: Cyan
        {1.0f, 0.5f, 0.0f},   // 6: Orange
        {0.5f, 0.0f, 1.0f},   // 7: Purple
        {1.0f, 0.75f, 0.8f},  // 8: Pink
        {0.5f, 0.8f, 1.0f}    // 9: Light Blue
    };
    constexpr int palette_size = static_cast<int>(sizeof(palette) / sizeof(palette[0]));

    for (size_t i = 0; i < all_waypoints_from_global_planner_.size(); i++)
    {
            // Create waypoint marker for main path from the global planner
            visualization_msgs::msg::Marker waypoint_marker;
            waypoint_marker.header.frame_id = "map";
            waypoint_marker.header.stamp = this->now();
            waypoint_marker.ns = "global_planner";
            waypoint_marker.id = i;
            waypoint_marker.type = visualization_msgs::msg::Marker::ARROW;
            waypoint_marker.action = visualization_msgs::msg::Marker::ADD;

            waypoint_marker.color.a = 0.8;

            // Color based on lane_sequence_id (mod palette size).
            const int seq_id = all_waypoints_from_global_planner_[i].lane_sequence_id;
            const int color_index = ((seq_id % palette_size) + palette_size) % palette_size;
            waypoint_marker.color.r = palette[color_index].r;
            waypoint_marker.color.g = palette[color_index].g;
            waypoint_marker.color.b = palette[color_index].b;

            waypoint_marker.pose.position.x = all_waypoints_from_global_planner_[i].x;
            waypoint_marker.pose.position.y = all_waypoints_from_global_planner_[i].y;
            waypoint_marker.pose.position.z = 0.0;

            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, all_waypoints_from_global_planner_[i].heading);
            waypoint_marker.pose.orientation.x = quaternion.x();
            waypoint_marker.pose.orientation.y = quaternion.y();
            waypoint_marker.pose.orientation.z = quaternion.z();
            waypoint_marker.pose.orientation.w = quaternion.w();

            waypoint_marker.scale.x = 0.6; // Arrow length
            waypoint_marker.scale.y = 0.2; // Arrow width
            waypoint_marker.scale.z = 0.2; // Arrow height

            global_planner_markers_.markers.push_back(waypoint_marker);
            
            // Create text marker for lane_sequence_id
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = this->now();
            text_marker.ns = "global_planner_text";
            text_marker.id = i;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Position text above the arrow
            text_marker.pose.position.x = all_waypoints_from_global_planner_[i].x;
            text_marker.pose.position.y = all_waypoints_from_global_planner_[i].y;
            text_marker.pose.position.z = 0.5; // Above the arrow
            
            // Set text content to lane_sequence_id
            text_marker.text = std::to_string(all_waypoints_from_global_planner_[i].lane_sequence_id);
            
            // Text styling
            text_marker.scale.z = 0.3; // Text size
            text_marker.color.a = 1.0;
            text_marker.color.r = 1.0; // White text
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            
            global_planner_markers_.markers.push_back(text_marker);
    }
    global_planner_publisher_->publish(global_planner_markers_);
}

// =============================
// publish the global planner occupancy grid
// =============================
void path_planning::publishGlobalPlannerOccupancyGrid()
{
    RCLCPP_INFO(this->get_logger(),
        "Publishing global planner occupancy grid (%ux%u, resolution=%.4fm)",
        global_planner_occupancy_grid_.info.width,
        global_planner_occupancy_grid_.info.height,
        static_cast<double>(global_planner_occupancy_grid_.info.resolution));

    global_planner_occupancy_grid_publisher_->publish(global_planner_occupancy_grid_);
}

// =============================
// map combination & rescale for put obstacles in the global map
// =============================
void path_planning::obstacle_info_callback(const path_planning_dynamic::msg::ObstacleCollection::SharedPtr msg)
{
    if (!global_map_)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Global map is not available");
        return;
    }
    if (msg->obstacles.empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Obstacle collection is empty; planning with base map only");
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(),
            "Obstacle collection received with %zu obstacles", msg->obstacles.size());
    }
    getCurrentRobotState();
    publishGlobalPlanner();
    RCLCPP_DEBUG(this->get_logger(), "Path planning map combination update.");
    map_combination(msg);
}

cv::Mat path_planning::toMat(const nav_msgs::msg::OccupancyGrid &map)
{
    cv::Mat im(map.info.height, map.info.width, CV_8UC1);
    for (size_t i = 0; i < map.data.size(); i++)
    {
        if (map.data[i] == 0)
            im.data[i] = 254; // Free space
        else if (map.data[i] == 100)
            im.data[i] = 0; // Occupied space
        else
            im.data[i] = 205; // Unknown space
    }
    return im;
}

cv::Mat path_planning::rescaleChunk(const cv::Mat &chunk_mat, double scale_factor)
{
    cv::Mat rescaled_chunk;
    cv::resize(chunk_mat, rescaled_chunk, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);
    return rescaled_chunk;
}

// ---------- map_combination helpers (declarations in path_planning.hpp) ----------
void path_planning::worldToGrid(double wx, double wy,
                                const nav_msgs::msg::MapMetaData &info,
                                int &gx, int &gy)
{
    // Use floor so negative offsets land in the correct cell. static_cast<int>
    // truncates toward zero and produces off-by-one errors near the origin.
    gx = static_cast<int>(std::floor((wx - info.origin.position.x) / info.resolution));
    gy = static_cast<int>(std::floor((wy - info.origin.position.y) / info.resolution));
}

bool path_planning::isInsideGrid(int gx, int gy,
                                 const nav_msgs::msg::MapMetaData &info)
{
    return gx >= 0 && gx < static_cast<int>(info.width) &&
           gy >= 0 && gy < static_cast<int>(info.height);
}

bool path_planning::validateScaleFactor(double sf) const
{
    if (!(sf > 0.0) || !std::isfinite(sf)) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Invalid scale_factor %f; must be > 0 and finite. Skipping update.", sf);
        return false;
    }
    return true;
}

int path_planning::computeInflationCells() const
{
    if (!rescaled_chunk_ || rescaled_chunk_->info.resolution <= 0.0) {
        return std::max(0, obstacle_inflation_radius_cells_);
    }
    if (obstacle_inflation_radius_m_ > 0.0) {
        return std::max(0, static_cast<int>(std::round(
            obstacle_inflation_radius_m_ / rescaled_chunk_->info.resolution)));
    }
    return std::max(0, obstacle_inflation_radius_cells_);
}

bool path_planning::extractChunkAroundRobot(nav_msgs::msg::OccupancyGrid &chunk) const
{
    // Window origin slightly ahead of the robot to bias visibility forward.
    const double origin_world_x = car_state_->x + forward_distance * std::cos(car_state_->heading);
    const double origin_world_y = car_state_->y + forward_distance * std::sin(car_state_->heading);

    int car_x_grid = 0;
    int car_y_grid = 0;
    worldToGrid(origin_world_x, origin_world_y, global_map_->info, car_x_grid, car_y_grid);

    const int gw = static_cast<int>(global_map_->info.width);
    const int gh = static_cast<int>(global_map_->info.height);

    // Clamp BOTH bounds to [0, dimension]. Previously only one side was clamped,
    // which made min > max underflow when the robot was far outside the map.
    const int min_x = std::clamp(car_x_grid - chunk_radius, 0, gw);
    const int max_x = std::clamp(car_x_grid + chunk_radius, 0, gw);
    const int min_y = std::clamp(car_y_grid - chunk_radius, 0, gh);
    const int max_y = std::clamp(car_y_grid + chunk_radius, 0, gh);

    if (max_x <= min_x || max_y <= min_y) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Chunk window is empty (robot likely outside the global map). "
            "car_grid=(%d,%d), min=(%d,%d), max=(%d,%d), map=%dx%d",
            car_x_grid, car_y_grid, min_x, min_y, max_x, max_y, gw, gh);
        return false;
    }

    chunk.header = global_map_->header;
    chunk.info.resolution = global_map_->info.resolution;
    chunk.info.width = static_cast<uint32_t>(max_x - min_x);
    chunk.info.height = static_cast<uint32_t>(max_y - min_y);
    chunk.info.origin.position.x =
        global_map_->info.origin.position.x + min_x * global_map_->info.resolution;
    chunk.info.origin.position.y =
        global_map_->info.origin.position.y + min_y * global_map_->info.resolution;
    chunk.info.origin.position.z = car_state_->z;
    chunk.info.origin.orientation.w = 1.0;

    chunk.data.assign(static_cast<size_t>(chunk.info.width) * chunk.info.height, 0);
    for (int y = min_y; y < max_y; ++y) {
        for (int x = min_x; x < max_x; ++x) {
            const int global_index = y * gw + x;
            const int local_x = x - min_x;
            const int local_y = y - min_y;
            const int chunk_index = local_y * static_cast<int>(chunk.info.width) + local_x;
            chunk.data[chunk_index] = global_map_->data[global_index];
        }
    }
    return true;
}

bool path_planning::rescaleOccupancyGridChunk(const nav_msgs::msg::OccupancyGrid &chunk)
{
    if (chunk.info.width == 0 || chunk.info.height == 0 || chunk.info.resolution <= 0.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "rescaleOccupancyGridChunk: invalid input chunk (%ux%u, res=%f)",
            chunk.info.width, chunk.info.height, static_cast<double>(chunk.info.resolution));
        return false;
    }

    const cv::Mat chunk_mat = toMat(chunk);
    if (chunk_mat.empty()) {
        RCLCPP_WARN(this->get_logger(), "rescaleOccupancyGridChunk: empty input cv::Mat");
        return false;
    }

    const cv::Mat rescaled_chunk_mat = rescaleChunk(chunk_mat, scale_factor);
    if (rescaled_chunk_mat.empty() || rescaled_chunk_mat.cols <= 0 || rescaled_chunk_mat.rows <= 0) {
        RCLCPP_WARN(this->get_logger(), "rescaleOccupancyGridChunk: empty rescaled cv::Mat");
        return false;
    }

    const double new_resolution = chunk.info.resolution / scale_factor;
    if (!(new_resolution > 0.0) || !std::isfinite(new_resolution)) {
        RCLCPP_ERROR(this->get_logger(),
            "Computed rescaled resolution is invalid: %f (chunk_res=%f, scale=%f)",
            new_resolution, static_cast<double>(chunk.info.resolution), scale_factor);
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(),
        "Rescale: original_res=%.4fm scale=%.4f -> new_res=%.4fm | "
        "size_before=%ux%u size_after=%dx%d",
        static_cast<double>(chunk.info.resolution), scale_factor, new_resolution,
        chunk.info.width, chunk.info.height,
        rescaled_chunk_mat.cols, rescaled_chunk_mat.rows);

    rescaled_chunk_->header = global_map_->header;
    rescaled_chunk_->info.resolution = static_cast<float>(new_resolution);
    rescaled_chunk_->info.width = static_cast<uint32_t>(rescaled_chunk_mat.cols);
    rescaled_chunk_->info.height = static_cast<uint32_t>(rescaled_chunk_mat.rows);
    rescaled_chunk_->info.origin.position.x = chunk.info.origin.position.x;
    rescaled_chunk_->info.origin.position.y = chunk.info.origin.position.y;
    rescaled_chunk_->info.origin.position.z = car_state_->z;
    rescaled_chunk_->info.origin.orientation.w = 1.0;
    rescaled_chunk_->data.assign(
        static_cast<size_t>(rescaled_chunk_->info.width) * rescaled_chunk_->info.height, 0);

    // Convert OpenCV grayscale (254 free, 0 occupied, 205 unknown) back to ROS
    // occupancy values (0 free, 100 occupied, -1 unknown).
    const size_t total_cells = static_cast<size_t>(rescaled_chunk_mat.rows) *
                               static_cast<size_t>(rescaled_chunk_mat.cols);
    for (size_t i = 0; i < total_cells; i++) {
        const uint8_t v = rescaled_chunk_mat.data[i];
        if (v == 254) {
            rescaled_chunk_->data[i] = 0;
        } else if (v == 0) {
            rescaled_chunk_->data[i] = 100;
        } else {
            rescaled_chunk_->data[i] = -1;
        }
    }
    return true;
}

void path_planning::buildWindowMask(cv::Mat &window_mask,
                                    std::vector<cv::Point> &window_polygon) const
{
    const double res = rescaled_chunk_->info.resolution;
    const double cos_h = std::cos(car_state_->heading);
    const double sin_h = std::sin(car_state_->heading);

    int car_gx = 0, car_gy = 0;
    worldToGrid(car_state_->x, car_state_->y, rescaled_chunk_->info, car_gx, car_gy);

    const double half = square_size_m_ / 2.0;
    static constexpr std::pair<double, double> corners[] = {
        {-1.0, -1.0}, { 1.0, -1.0}, { 1.0,  1.0}, {-1.0,  1.0}
    };

    window_polygon.clear();
    window_polygon.reserve(4);
    for (const auto &c : corners) {
        const double cx = c.first * half;
        const double cy = c.second * half;
        const double rx = cx * cos_h - cy * sin_h;
        const double ry = cx * sin_h + cy * cos_h;
        const int gx = car_gx + static_cast<int>(std::round(rx / res));
        const int gy = car_gy + static_cast<int>(std::round(ry / res));
        window_polygon.emplace_back(gx, gy);
    }

    window_mask = cv::Mat(rescaled_chunk_->info.height, rescaled_chunk_->info.width,
                          CV_8UC1, cv::Scalar(0));
    cv::fillConvexPoly(window_mask, window_polygon, cv::Scalar(255));
}

// ---------- map_combination ----------
void path_planning::map_combination(const path_planning_dynamic::msg::ObstacleCollection::SharedPtr msg)
{
    if (!car_state_valid_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Skipping map_combination: robot pose not yet available (TF lookup failed).");
        return;
    }
    if (!global_map_ || global_map_->data.empty() ||
        global_map_->info.resolution <= 0.0 ||
        global_map_->info.width == 0 || global_map_->info.height == 0) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Skipping map_combination: global_map_ is missing or invalid (res=%f, %ux%u, data=%zu).",
            global_map_ ? static_cast<double>(global_map_->info.resolution) : 0.0,
            global_map_ ? global_map_->info.width : 0u,
            global_map_ ? global_map_->info.height : 0u,
            global_map_ ? global_map_->data.size() : 0);
        return;
    }
    if (!validateScaleFactor(scale_factor)) {
        return;
    }

    const auto init_time = std::chrono::system_clock::now();
    const auto current_stamp = this->now();
    rescaled_chunk_->data.clear();

    // 1) Extract a chunk of the global map around the robot.
    nav_msgs::msg::OccupancyGrid chunk;
    if (!extractChunkAroundRobot(chunk)) {
        return;
    }

    // 2) Rescale (cv::resize) and copy back to OccupancyGrid using the corrected resolution.
    if (!rescaleOccupancyGridChunk(chunk)) {
        return;
    }

    // 3) Local planning window (square_size_m_ centered ahead of the robot).
    cv::Mat window_mask;
    std::vector<cv::Point> window_polygon;
    buildWindowMask(window_mask, window_polygon);

    // 4) Seed the dynamic obstacle grid: -1 (unknown) everywhere, free (0) inside the window.
    nav_msgs::msg::OccupancyGrid dynamic_obstacle_grid = *rescaled_chunk_;
    dynamic_obstacle_grid.header.stamp = current_stamp;
    dynamic_obstacle_grid.data.assign(
        static_cast<size_t>(dynamic_obstacle_grid.info.width) * dynamic_obstacle_grid.info.height, -1);
    for (int y = 0; y < window_mask.rows; ++y) {
        for (int x = 0; x < window_mask.cols; ++x) {
            if (window_mask.at<uint8_t>(y, x) != 0) {
                dynamic_obstacle_grid.data[y * dynamic_obstacle_grid.info.width + x] = 0;
            }
        }
    }

    // 5) Carry the static base map forward; dynamic obstacles will be stamped on top
    //    of dynamic_global_obstacle_grid (global_map_ itself stays as the static base).
    nav_msgs::msg::OccupancyGrid dynamic_global_obstacle_grid = *global_map_;
    dynamic_global_obstacle_grid.header.stamp = current_stamp;

    // 6) Inflation radius: prefer meter-based parameter; convert to cells using the
    //    *corrected* rescaled resolution so behavior is resolution-stable.
    const int inflation_radius = computeInflationCells();
    constexpr int value_to_mark = 100;

    RCLCPP_DEBUG(this->get_logger(),
        "Inflation: meters=%.4f -> cells=%d (rescaled_res=%.4fm)",
        obstacle_inflation_radius_m_, inflation_radius,
        static_cast<double>(rescaled_chunk_->info.resolution));

    // Stamp value at (x,y) in the local chunk and project to the dynamic global grid.
    auto mark_grid = [&](int x, int y, int value) {
        if (!isInsideGrid(x, y, rescaled_chunk_->info)) {
            return;
        }
        if (window_mask.at<uint8_t>(y, x) == 0) {
            return;
        }
        rescaled_chunk_->data[y * rescaled_chunk_->info.width + x] = value;
        dynamic_obstacle_grid.data[y * dynamic_obstacle_grid.info.width + x] = value;

        const double world_x =
            rescaled_chunk_->info.origin.position.x +
            (static_cast<double>(x) + 0.5) * rescaled_chunk_->info.resolution;
        const double world_y =
            rescaled_chunk_->info.origin.position.y +
            (static_cast<double>(y) + 0.5) * rescaled_chunk_->info.resolution;

        int gx = 0, gy = 0;
        worldToGrid(world_x, world_y, dynamic_global_obstacle_grid.info, gx, gy);
        if (isInsideGrid(gx, gy, dynamic_global_obstacle_grid.info)) {
            dynamic_global_obstacle_grid
                .data[gy * dynamic_global_obstacle_grid.info.width + gx] = value;
        }
    };

    auto inflate_point = [&](int x, int y, int radius, int value) {
        if (radius <= 0) {
            mark_grid(x, y, value);
            return;
        }
        const int r2 = radius * radius;
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                if (dx * dx + dy * dy <= r2) {
                    mark_grid(x + dx, y + dy, value);
                }
            }
        }
    };

    auto draw_inflated_line = [&](int x0, int y0, int x1, int y1, int radius, int value) {
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int n = 1 + dx + dy;
        const int x_inc = (x1 > x0) ? 1 : -1;
        const int y_inc = (y1 > y0) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; --n) {
            inflate_point(x0, y0, radius, value);
            if (error > 0) {
                x0 += x_inc;
                error -= dy;
            } else {
                y0 += y_inc;
                error += dx;
            }
        }
    };

    const std::string map_frame = dynamic_global_obstacle_grid.header.frame_id.empty()
        ? global_planner_frame_id_
        : dynamic_global_obstacle_grid.header.frame_id;
    geometry_msgs::msg::TransformStamped obstacle_to_map_tf;
    if (!msg->obstacles.empty()) {
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Skipping map_combination: obstacle collection has an empty frame_id.");
            return;
        }
        try {
            obstacle_to_map_tf = tf2_buffer.lookupTransform(
                map_frame, msg->header.frame_id, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Skipping map_combination: transform error (%s <- %s): %s",
                map_frame.c_str(), msg->header.frame_id.c_str(), ex.what());
            return;
        }
    }

    // Even-odd ray-casting test. Returns false for degenerate polygons.
    auto point_in_polygon = [](int x, int y, const std::vector<std::pair<int, int>> &polygon) -> bool {
        if (polygon.size() < 3) {
            return false;
        }
        bool inside = false;
        int j = static_cast<int>(polygon.size()) - 1;
        for (int i = 0; i < static_cast<int>(polygon.size()); i++) {
            const auto &pi = polygon[i];
            const auto &pj = polygon[j];
            // The strict inequality on y guards against div-by-zero when
            // both endpoints share the scan-line; collinear edges are ignored.
            if (((pi.second > y) != (pj.second > y)) &&
                (x < (pj.first - pi.first) * (y - pi.second) /
                     (pj.second - pi.second) + pi.first)) {
                inside = !inside;
            }
            j = i;
        }
        return inside;
    };

    auto fill_obstacle_interior = [&](const std::vector<std::pair<int, int>> &polygon_vertices,
                                       int fill_value) {
        if (polygon_vertices.size() < 3) {
            return;
        }
        int min_x = polygon_vertices[0].first, max_x = polygon_vertices[0].first;
        int min_y = polygon_vertices[0].second, max_y = polygon_vertices[0].second;
        for (const auto &v : polygon_vertices) {
            min_x = std::min(min_x, v.first);
            max_x = std::max(max_x, v.first);
            min_y = std::min(min_y, v.second);
            max_y = std::max(max_y, v.second);
        }
        if (max_x == min_x && max_y == min_y) {
            return; // degenerate single-point polygon
        }
        min_x = std::max(0, min_x);
        max_x = std::min(static_cast<int>(rescaled_chunk_->info.width) - 1, max_x);
        min_y = std::max(0, min_y);
        max_y = std::min(static_cast<int>(rescaled_chunk_->info.height) - 1, max_y);

        for (int y = min_y; y <= max_y; ++y) {
            for (int x = min_x; x <= max_x; ++x) {
                if (point_in_polygon(x, y, polygon_vertices)) {
                    mark_grid(x, y, fill_value);
                }
            }
        }
    };

    for (const auto &obstacle : msg->obstacles) {
        geometry_msgs::msg::Polygon map_polygon;
        tf2::doTransform(obstacle.polygon, map_polygon, obstacle_to_map_tf);
        const auto &pts = map_polygon.points;
        if (pts.size() < 3) {
            RCLCPP_DEBUG(this->get_logger(),
                "Skipping obstacle %u with degenerate polygon (%zu points)",
                obstacle.id, pts.size());
            continue;
        }

        std::vector<std::pair<int, int>> polygon_vertices;
        polygon_vertices.reserve(pts.size());

        for (size_t j = 0; j < pts.size(); ++j) {
            const auto &current_point_map = pts[j];
            const auto &next_point_map = pts[(j + 1) % pts.size()];

            int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
            worldToGrid(current_point_map.x, current_point_map.y, rescaled_chunk_->info, x0, y0);
            worldToGrid(next_point_map.x, next_point_map.y, rescaled_chunk_->info, x1, y1);

            polygon_vertices.emplace_back(x0, y0);
            draw_inflated_line(x0, y0, x1, y1, inflation_radius, value_to_mark);
        }

        fill_obstacle_interior(polygon_vertices, value_to_mark);
    }

    // Crop the published dynamic obstacle grid to the window bounding rectangle.
    nav_msgs::msg::OccupancyGrid published_dynamic_obstacle_grid = dynamic_obstacle_grid;
    cv::Rect window_bounds = cv::boundingRect(window_polygon);
    window_bounds &= cv::Rect(0, 0,
                              static_cast<int>(dynamic_obstacle_grid.info.width),
                              static_cast<int>(dynamic_obstacle_grid.info.height));
    if (window_bounds.width > 0 && window_bounds.height > 0) {
        published_dynamic_obstacle_grid.info.width = static_cast<uint32_t>(window_bounds.width);
        published_dynamic_obstacle_grid.info.height = static_cast<uint32_t>(window_bounds.height);
        published_dynamic_obstacle_grid.info.origin.position.x +=
            static_cast<double>(window_bounds.x) * dynamic_obstacle_grid.info.resolution;
        published_dynamic_obstacle_grid.info.origin.position.y +=
            static_cast<double>(window_bounds.y) * dynamic_obstacle_grid.info.resolution;
        published_dynamic_obstacle_grid.data.assign(
            static_cast<size_t>(published_dynamic_obstacle_grid.info.width) *
                published_dynamic_obstacle_grid.info.height, -1);

        for (int y = 0; y < window_bounds.height; ++y) {
            for (int x = 0; x < window_bounds.width; ++x) {
                const int src_x = window_bounds.x + x;
                const int src_y = window_bounds.y + y;
                published_dynamic_obstacle_grid.data[
                    y * published_dynamic_obstacle_grid.info.width + x] =
                    dynamic_obstacle_grid.data[src_y * dynamic_obstacle_grid.info.width + src_x];
            }
        }
    }

    buildDistanceField();
    buildWaypointDistanceFields();
    grid_map_ = std::make_shared<Grid_map>(*rescaled_chunk_);
    grid_map_->setVehicleFootprint(vehicle_footprint_);

    global_planner_occupancy_grid_ = dynamic_global_obstacle_grid;
    occupancy_grid_pub_test_->publish(published_dynamic_obstacle_grid);
    global_planner_occupancy_grid_publisher_->publish(global_planner_occupancy_grid_);

    TreeFlat flat_map;
    const int best_map = generateTrajectoryTree_AStar_flat_map_with_waypoints(*car_state_, flat_map);
    publishBestPathFromFlat(flat_map, best_map, 2);
    publishAllPathsFromFlat(flat_map);
    publishTrajectoryPath(flat_map, best_map);

    const auto end_time = std::chrono::system_clock::now();
    const auto duration_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    RCLCPP_DEBUG(this->get_logger(), "Execution time for path selection: %ld ms", duration_ms);
}

  
// =============================
// generate the trajectory based on the A* algorithm
// =============================
int path_planning::generateTrajectoryTree_AStar_flat_map_with_waypoints(const State& root_state, TreeFlat& out)
{
    return generateTrajectoryTreeImpl(root_state, out, true);
}

int path_planning::generateTrajectoryTreeImpl(const State& root_state, TreeFlat& out, const bool use_waypoints)
{
    out.nodes.clear();
    out.leaves.clear();

    if (!grid_map_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Skipping A* expansion: grid_map_ is null (no obstacle update received yet).");
        return -1;
    }
    if (!kinematic_model_) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Skipping A* expansion: kinematic_model_ is null.");
        return -1;
    }
    if (motion_primitives_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Skipping A* expansion: motion_primitives_ is empty.");
        return -1;
    }
    if (pathLength <= 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Skipping A* expansion: pathLength <= 0 (got %d).", pathLength);
        return -1;
    }

    const int B = std::max(1, static_cast<int>(motion_primitives_.size()));
    const int D = std::max(0, tree_depth);
    const int EFFECTIVE_DEPTH = (D > 0) ? (D - 1) : 0;

    // Anchor the forward/lateral/heading cost to the GLOBAL ROUTE tangent near
    // the robot instead of the robot's (possibly misaligned) current heading.
    // The previous behaviour froze the reference at root_state.heading, so any
    // route that started behind/beside the robot incurred a forward+heading
    // penalty with no offsetting reward — the best leaf collapsed back to the
    // root pose and the robot never turned to pick the route up. Using the
    // nearest priority-1 (centerline) waypoint's heading lets the planner choose
    // in-place rotation to align with the route. Falls back to the robot heading
    // when no route waypoints are available.
    double ref_heading = root_state.heading;
    if (use_waypoints) {
        double best_d2 = std::numeric_limits<double>::infinity();
        for (const auto& wp : all_waypoints_from_global_planner_) {
            if (wp.priority != 1) continue;   // priority-1 = main centerline
            const double ddx = wp.x - root_state.x;
            const double ddy = wp.y - root_state.y;
            const double d2 = ddx * ddx + ddy * ddy;
            if (d2 < best_d2) {
                best_d2 = d2;
                ref_heading = wp.heading;
            }
        }
    }

    const double cs0 = std::cos(ref_heading);
    const double ss0 = std::sin(ref_heading);

    size_t max_nodes = 1;
    size_t powB = 1;
    for (int d = 0; d < EFFECTIVE_DEPTH; ++d) {
        powB *= static_cast<size_t>(B);
        max_nodes += powB;
    }
    max_nodes = std::min(max_nodes, static_cast<size_t>(500000));
    out.nodes.reserve(max_nodes);

    FlatNode root;
    root.state = root_state;
    root.parent = -1;
    root.primitive_index = -1;
    root.depth = 0;
    root.cost = 0.0;
    out.nodes.push_back(root);

    int best_goal_idx = -1;
    double best_goal_cost = std::numeric_limits<double>::infinity();

    std::priority_queue<PQItem> open;
    std::unordered_map<LatticeKey, double, LatticeKeyHash> best_g;

    auto stateKey = [&](const State& s) -> LatticeKey {
        return LatticeKey{static_cast<int>(s.gridx), static_cast<int>(s.gridy), heading_bin(s.heading)};
    };

    auto h_lower_bound = [&](int depth) -> double {
        const int remaining_segments = EFFECTIVE_DEPTH - depth;
        if (remaining_segments <= 0) {
            return 0.0;
        }
        const int remaining_steps = remaining_segments * pathLength;
        return -W_FORWARD * (remaining_steps * kinematic_model_->maxForwardStep());
    };

    auto sample_wp_dist = [&](int gx, int gy) -> double {
        if (!use_waypoints) {
            return 0.0;
        }
        if (has_wp1_ && !dist_wp1_m_.empty()) {
            if (gy >= 0 && gy < dist_wp1_m_.rows && gx >= 0 && gx < dist_wp1_m_.cols) {
                return static_cast<double>(dist_wp1_m_.at<float>(gy, gx)) * W_WP1;
            }
            return 0.0;
        }
        if (has_wp2_ && !dist_wp2_m_.empty()) {
            if (gy >= 0 && gy < dist_wp2_m_.rows && gx >= 0 && gx < dist_wp2_m_.cols) {
                return static_cast<double>(dist_wp2_m_.at<float>(gy, gx)) * W_WP2;
            }
            return 0.0;
        }
        return 0.0;
    };

    {
        const LatticeKey k = stateKey(root.state);
        best_g[k] = 0.0;
        open.push(PQItem{0, h_lower_bound(0), 0.0});
    }

    auto expand_one = [&](int parent_idx, size_t primitive_idx) -> int {
        const FlatNode& parent = out.nodes[parent_idx];
        const MotionPrimitive& primitive = motion_primitives_[primitive_idx];
        RolloutResult rollout =
            kinematic_model_->rollout(parent.state, primitive, pathLength);

        if (rollout.samples.empty()) {
            return -1;
        }

        double obs_pen_sum = 0.0;
        double wp_pen_sum = 0.0;

        for (auto& ns : rollout.samples) {
            ns.z = parent.state.z;
            auto cell = grid_map_->toCellID(ns);
            ns.gridx = std::get<0>(cell);
            ns.gridy = std::get<1>(cell);

            if (grid_map_->isSingleStateInCollisionImproved(ns)) {
              return -1;
            }

            const double d = clearanceMeters(static_cast<int>(ns.gridx), static_cast<int>(ns.gridy));
            if (d < SAFE_CLEAR) {
                obs_pen_sum += (SAFE_CLEAR - d);
            }

            if (use_waypoints && d < SAFE_CLEAR * 0.6) {
                return -1;
            }

            wp_pen_sum += sample_wp_dist(static_cast<int>(ns.gridx), static_cast<int>(ns.gridy));
        }

        FlatNode child;
        child.state = rollout.samples.back();
        child.parent = parent_idx;
        child.primitive_index = static_cast<int>(primitive_idx);
        child.depth = parent.depth + 1;
        child.segment_samples = std::move(rollout.samples);

        const MotionPrimitive* previous_primitive =
            parent.primitive_index >= 0 ? &motion_primitives_[parent.primitive_index] : nullptr;
        const double steer_pen =
            W_STEER * kinematic_model_->controlEffort(primitive);
        const double dsteer_pen =
            W_DSTEER * kinematic_model_->smoothnessCost(previous_primitive, primitive);

        const double dx = child.state.x - parent.state.x;
        const double dy = child.state.y - parent.state.y;
        const double forward_inc = dx * cs0 + dy * ss0;

        const double obs_pen = W_OBS * (obs_pen_sum / std::max(1, pathLength));
        const double wp_pen =
            use_waypoints ? (wp_pen_sum / std::max(1, pathLength)) : 0.0;

        double straight_penalty = 0.0;
        if (use_waypoints && std::fabs(primitive.angular_step) < 1e-6 &&
            std::fabs(primitive.steering_angle) < 1e-6 && forward_inc > 0.0) {
            const double avg_clearance = obs_pen_sum / std::max(1, pathLength);
            if (avg_clearance > 0.1) {
                straight_penalty = 2.0;
            }
        }

        const double g_child = parent.cost + steer_pen + dsteer_pen + obs_pen +
                               wp_pen + straight_penalty -
                               W_FORWARD * forward_inc;

        child.cost = g_child;

        const LatticeKey ck = stateKey(child.state);
        auto it = best_g.find(ck);
        if (it != best_g.end() && g_child >= it->second - 1e-12) {
            return -1;
        }
        best_g[ck] = g_child;

        out.nodes.push_back(std::move(child));
        return static_cast<int>(out.nodes.size()) - 1;
    };

    while (!open.empty()) {
        const PQItem cur = open.top();
        open.pop();

        const int idx = cur.idx;
        const auto& fn = out.nodes[idx];
        const double g = fn.cost;
        const int d = fn.depth;

        if (std::fabs(g - cur.g_copy) > 1e-12) {
            continue;
        }

        if (d == EFFECTIVE_DEPTH) {
            const double dx = fn.state.x - root_state.x;
            const double dy = fn.state.y - root_state.y;
            const double lateral = -dx * ss0 + dy * cs0;
            const double head_err =
                std::fabs(wrapAngle(fn.state.heading - ref_heading));

            const double total =
                g + W_LAT * std::fabs(lateral) + W_HEAD * head_err;

            if (total < best_goal_cost) {
                best_goal_cost = total;
                best_goal_idx = idx;
            }

            if (!open.empty() && open.top().f_est >= best_goal_cost - 1e-12) {
                break;
            }
            continue;
        }

        bool produced_child = false;
        for (size_t primitive_idx = 0; primitive_idx < motion_primitives_.size();
             ++primitive_idx) {
            const int child_idx = expand_one(idx, primitive_idx);
            if (child_idx < 0) {
                continue;
            }
            produced_child = true;

            const auto& ch = out.nodes[child_idx];
            open.push(PQItem{child_idx, ch.cost + h_lower_bound(ch.depth),
                             ch.cost});
        }

        if (!produced_child) {
            const double dx = fn.state.x - root_state.x;
            const double dy = fn.state.y - root_state.y;
            const double lateral = -dx * ss0 + dy * cs0;
            const double head_err =
                std::fabs(wrapAngle(fn.state.heading - ref_heading));

            const double total =
                g + W_LAT * std::fabs(lateral) + W_HEAD * head_err;

            if (total < best_goal_cost) {
                best_goal_cost = total;
                best_goal_idx = idx;
            }
            if (!open.empty() && open.top().f_est >= best_goal_cost - 1e-12) {
                break;
            }
        }
    }

    out.leaves.clear();
    for (size_t i = 0; i < out.nodes.size(); ++i) {
        if (out.nodes[i].depth == EFFECTIVE_DEPTH) {
            out.leaves.push_back(static_cast<int>(i));
        }
    }

    if (best_goal_idx >= 0) {
        const bool is_listed = std::find(out.leaves.begin(), out.leaves.end(),
                                         best_goal_idx) != out.leaves.end();
        if (!is_listed) {
            out.leaves.push_back(best_goal_idx);
        }
    }

    return best_goal_idx;
}


// =============================
//  helper functions for the A* algorithm
// =============================
inline void path_planning::build_chain_indices(
    const TreeFlat& flat, int leaf_idx, std::vector<int>& chain) const
{
    chain.clear();
    for (int i = leaf_idx; i != -1; i = flat.nodes[i].parent) chain.push_back(i);
    std::reverse(chain.begin(), chain.end()); // root -> leaf
}

void path_planning::buildDistanceField()
{
    if (!rescaled_chunk_ || rescaled_chunk_->data.empty()) {
        dist_m_.release();
        return;
    }

    const int H = static_cast<int>(rescaled_chunk_->info.height);
    const int W = static_cast<int>(rescaled_chunk_->info.width);
    const double res = rescaled_chunk_->info.resolution; // 0.2 in your setup

    // Build binary image for distance transform: free=255, else=0 (occupied OR unknown)
    cv::Mat bin(H, W, CV_8UC1);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const int8_t v = rescaled_chunk_->data[y * W + x];
            // Your convention: 0=free, 100=occupied, -1=unknown
            bin.at<uint8_t>(y, x) = (v == 0) ? 255 : 0;
        }
    }

    // Distance transform in pixels
    cv::Mat dist_px;
    cv::distanceTransform(bin, dist_px, cv::DIST_L2, 3);

    // Convert pixels to meters
    dist_m_.create(H, W, CV_32FC1);
    const float scale = static_cast<float>(res);
    for (int y = 0; y < H; ++y) {
        const float* src = dist_px.ptr<float>(y);
        float*       dst = dist_m_.ptr<float>(y);
        for (int x = 0; x < W; ++x) dst[x] = src[x] * scale;
    }
}

inline double path_planning::clearanceMeters(int gx, int gy) const
{
    if (dist_m_.empty()) return 0.0;
    if (gy < 0 || gy >= dist_m_.rows || gx < 0 || gx >= dist_m_.cols) return 0.0;
    return static_cast<double>(dist_m_.at<float>(gy, gx));
}

void path_planning::buildWaypointDistanceFields()
{
    dist_wp1_m_.release();
    dist_wp2_m_.release();
    has_wp1_ = false;
    has_wp2_ = false;

    if (!rescaled_chunk_ || rescaled_chunk_->data.empty()) return;

    const int H = static_cast<int>(rescaled_chunk_->info.height);
    const int W = static_cast<int>(rescaled_chunk_->info.width);
    const double res = rescaled_chunk_->info.resolution;

    // Binary canvases for each priority: 255 where path pixels live, 0 elsewhere
    cv::Mat bin1(H, W, CV_8UC1, cv::Scalar(0));
    cv::Mat bin2(H, W, CV_8UC1, cv::Scalar(0));

    auto worldToGrid = [&](double x, double y, int& gx, int& gy) {
        // Floor (not truncate) so negative offsets land in the correct cell.
        gx = static_cast<int>(std::floor((x - rescaled_chunk_->info.origin.position.x) / res));
        gy = static_cast<int>(std::floor((y - rescaled_chunk_->info.origin.position.y) / res));
    };

    // Helper to draw a thick line in grid space
    auto drawThickLine = [&](cv::Mat& img, int x0, int y0, int x1, int y1, int radius) {
        cv::LineIterator it(img, cv::Point(x0, y0), cv::Point(x1, y1));
        for (int i = 0; i < it.count; ++i, ++it) {
            cv::circle(img, it.pos(), radius, cv::Scalar(255), cv::FILLED);
        }
    };

    // Group consecutive waypoints by priority and draw lines between neighbors
    auto drawByPriority = [&](int prio, cv::Mat& canvas, bool& has_any) {
        std::vector<cv::Point> pts;
        pts.reserve(all_waypoints_from_global_planner_.size());

        // Collect all in-chunk points for this priority
        for (const auto& p : all_waypoints_from_global_planner_) {
            if (p.priority != prio) continue;
            int gx, gy;
            worldToGrid(p.x, p.y, gx, gy);
            if (gx >= 0 && gx < W && gy >= 0 && gy < H) {
                pts.emplace_back(gx, gy);
            }
        }

        // Draw segments between consecutive in-chunk points
        if (pts.size() >= 1) {
            has_any = true;
            const int rad = std::max(1, (int)std::round(WP_STROKE_RADIUS_CELLS));
            // Densify by connecting consecutive points
            for (size_t i = 1; i < pts.size(); ++i) {
                drawThickLine(canvas, pts[i-1].x, pts[i-1].y, pts[i].x, pts[i].y, rad);
            }
            // Also mark isolated singletons
            for (const auto& q : pts) {
                cv::circle(canvas, q, rad, cv::Scalar(255), cv::FILLED);
            }
        }
    };

    drawByPriority(1, bin1, has_wp1_);
    drawByPriority(2, bin2, has_wp2_);

    // If there is no prio-1 in chunk, keep bin1 empty; same for prio-2.
    auto toMetersDist = [&](const cv::Mat& bin, cv::Mat& out_m) {
        if (cv::countNonZero(bin) == 0) {
            out_m.release(); // no geometry to attract to
            return;
        }
        cv::Mat inv; // distanceTransform wants non-zero free area as source; we want distance TO the path
        // Build "feature" mask: 255 at path pixels, 0 elsewhere; we want distance TO those features,
        // so invert to a mask where features are 0 and background is 255, then DT that.
        cv::Mat feat = 255 - bin;

        cv::Mat dist_px;
        cv::distanceTransform(feat, dist_px, cv::DIST_L2, 3);

        out_m.create(bin.rows, bin.cols, CV_32FC1);
        const float scale = static_cast<float>(res);
        for (int y = 0; y < bin.rows; ++y) {
            const float* src = dist_px.ptr<float>(y);
            float*       dst = out_m.ptr<float>(y);
            for (int x = 0; x < bin.cols; ++x) dst[x] = src[x] * scale;
        }
    };

    toMetersDist(bin1, dist_wp1_m_);
    toMetersDist(bin2, dist_wp2_m_);

    // Reconcile availability flags with actual outputs
    has_wp1_ = has_wp1_ && !dist_wp1_m_.empty();
    has_wp2_ = has_wp2_ && !dist_wp2_m_.empty();
}


// =============================
// publish the trajectory
// =============================
void path_planning::publishBestPathFromFlat(const TreeFlat& flat, int leaf_idx, int color_idx)
{
    if (color_idx == 1) 
    {
        if (leaf_idx < 0 || real_trajectories_pub_->get_subscription_count() == 0) return;
    }
    else if (color_idx == 2)
    {
        if (leaf_idx < 0 || real_trajectories_pub_2->get_subscription_count() == 0) return;
    }


    // Build chain root->leaf
    std::vector<int> chain;
    build_chain_indices(flat, leaf_idx, chain);

    visualization_msgs::msg::MarkerArray msg;

    // clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = this->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.ns = "real_trajectories"; msg.markers.push_back(clear);
    clear.ns = "real_endpoints";     msg.markers.push_back(clear);
    clear.ns = "real_trajectory_labels"; msg.markers.push_back(clear);

    // line marker
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = this->now();
    line.ns = "real_trajectories";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.1;

    // chose btw to color to know wich implementation is used
    if (color_idx == 0) {
        line.color.r = 1.0; line.color.g = 0.0; line.color.b = 1.0;
    } else if (color_idx == 1) {
        line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0;
    } else if (color_idx == 2) {
        line.color.r = 0.0; line.color.g = 0.0; line.color.b = 1.0;
    }
    else {
        line.color.r = 1.0; line.color.g = 0.0; line.color.b = 1.0;
    }

    line.color.a = 1.0;

    // start at the true root pose so the polyline includes the origin point
    const State &root_state = flat.nodes[chain.front()].state;
    {
        geometry_msgs::msg::Point p;
        p.x = root_state.x; p.y = root_state.y; p.z = root_state.z;
        line.points.push_back(p);
    }

    State end_state = root_state;
    for (size_t k = 1; k < chain.size(); ++k)
    {
        const auto& fn = flat.nodes[chain[k]];
        for (const auto& sample : fn.segment_samples) {
            geometry_msgs::msg::Point p;
            p.x = sample.x;
            p.y = sample.y;
            p.z = sample.z;
            line.points.push_back(p);
            end_state = sample;
        }
    }

    msg.markers.push_back(line);

    // endpoint sphere
    visualization_msgs::msg::Marker endpoint;
    endpoint.header.frame_id = "map";
    endpoint.header.stamp = this->now();
    endpoint.ns = "real_endpoints";
    endpoint.id = 1;
    endpoint.type = visualization_msgs::msg::Marker::SPHERE;
    endpoint.action = visualization_msgs::msg::Marker::ADD;
    endpoint.pose.position.x = end_state.x;
    endpoint.pose.position.y = end_state.y;
    endpoint.pose.position.z = end_state.z + 0.2;
    endpoint.scale.x = 0.2; endpoint.scale.y = 0.2; endpoint.scale.z = 0.2;
    endpoint.color = line.color; endpoint.color.a = 0.8;
    msg.markers.push_back(endpoint);

    if (color_idx == 1) 
    {
        real_trajectories_pub_->publish(msg);
    }
    else if (color_idx == 2)
    {
        real_trajectories_pub_2->publish(msg);
    }
}


void path_planning::publishAllPathsFromFlat(const TreeFlat& flat)
{
    if (all_paths_pub_->get_subscription_count() == 0) return;

    visualization_msgs::msg::MarkerArray msg;

    // Clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = this->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.ns = "all_paths";
    msg.markers.push_back(clear);

    // Publish all paths from root to each leaf
    for (size_t leaf_idx = 0; leaf_idx < flat.leaves.size(); ++leaf_idx)
    {
        int leaf = flat.leaves[leaf_idx];
        if (leaf < 0) continue;

        // Build chain root->leaf
        std::vector<int> chain;
        build_chain_indices(flat, leaf, chain);

        // Create line marker for this path
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = this->now();
        line.ns = "all_paths";
        line.id = static_cast<int>(leaf_idx);
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.05; // Thinner lines for all paths

        // Color based on path index (cycle through colors)
        int color_idx = leaf_idx % 6;
        switch (color_idx) {
            case 0: line.color.r = 1.0; line.color.g = 0.0; line.color.b = 0.0; break; // Red
            case 1: line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0; break; // Green
            case 2: line.color.r = 0.0; line.color.g = 0.0; line.color.b = 1.0; break; // Blue
            case 3: line.color.r = 1.0; line.color.g = 1.0; line.color.b = 0.0; break; // Yellow
            case 4: line.color.r = 1.0; line.color.g = 0.0; line.color.b = 1.0; break; // Magenta
            case 5: line.color.r = 0.0; line.color.g = 1.0; line.color.b = 1.0; break; // Cyan
        }
        line.color.a = 0.7; // Semi-transparent

        // Start at the root pose
        const State &root_state = flat.nodes[chain.front()].state;
        {
            geometry_msgs::msg::Point p;
            p.x = root_state.x; p.y = root_state.y; p.z = root_state.z;
            line.points.push_back(p);
        }

        for (size_t k = 1; k < chain.size(); ++k)
        {
            const auto& fn = flat.nodes[chain[k]];
            for (const auto& sample : fn.segment_samples) {
                geometry_msgs::msg::Point p;
                p.x = sample.x;
                p.y = sample.y;
                p.z = sample.z + 0.2;
                line.points.push_back(p);
            }
        }

        msg.markers.push_back(line);
    }

    all_paths_pub_->publish(msg);
}


void path_planning::publishTrajectoryPath(const TreeFlat& flat, int leaf_idx)
{
    if (sdv_trajectory_pub_->get_subscription_count() == 0) return;
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    if (leaf_idx < 0) {
        sdv_trajectory_pub_->publish(path_msg);
        return;
    }

    // Build chain root->leaf
    std::vector<int> chain;
    build_chain_indices(flat, leaf_idx, chain);

    // Start at the root pose
    const State &root_state = flat.nodes[chain.front()].state;
    
    // Add the starting point
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "map";
    start_pose.header.stamp = path_msg.header.stamp;
    start_pose.pose.position.x = root_state.x;
    start_pose.pose.position.y = root_state.y;
    start_pose.pose.position.z = root_state.z;
    
    // Convert heading to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, root_state.heading);
    start_pose.pose.orientation.x = q.x();
    start_pose.pose.orientation.y = q.y();
    start_pose.pose.orientation.z = q.z();
    start_pose.pose.orientation.w = q.w();
    
    path_msg.poses.push_back(start_pose);

    for (size_t k = 1; k < chain.size(); ++k)
    {
        const auto& fn = flat.nodes[chain[k]];
        for (const auto& sample : fn.segment_samples) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = path_msg.header.stamp;
            pose.pose.position.x = sample.x;
            pose.pose.position.y = sample.y;
            pose.pose.position.z = sample.z + 0.2;

            tf2::Quaternion sample_q;
            sample_q.setRPY(0, 0, sample.heading);
            pose.pose.orientation.x = sample_q.x();
            pose.pose.orientation.y = sample_q.y();
            pose.pose.orientation.z = sample_q.z();
            pose.pose.orientation.w = sample_q.w();

            path_msg.poses.push_back(pose);
        }
    }

    sdv_trajectory_pub_->publish(path_msg);
}


// ─── Action Server Callbacks ──────────────────────────────────────────

rclcpp_action::GoalResponse path_planning::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToGoal::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Action Server: Received goal request for location '%s'", goal->location_name.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse path_planning::handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Action Server: Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void path_planning::handle_accepted(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
{
    std::lock_guard<std::mutex> lock(action_server_mutex_);
    if (active_goal_handle_ && active_goal_handle_->is_active()) {
        RCLCPP_INFO(this->get_logger(), "Action Server: Preempting active goal");
        auto result = std::make_shared<NavigateToGoal::Result>();
        result->success = false;
        result->message = "Preempted by new goal";
        active_goal_handle_->abort(result);
    }
    active_goal_handle_ = goal_handle;

    // Capture shared ownership so the node cannot be destroyed under the thread.
    auto self = std::static_pointer_cast<path_planning>(shared_from_this());
    std::thread([self, goal_handle]() {
        self->execute_navigation(goal_handle);
    }).detach();
}

void path_planning::execute_navigation(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "Action Server: Starting navigation execution to '%s' (lanelet: '%s')",
                goal->location_name.c_str(), goal->lanelet_name.c_str());

    // 1. Rebuild global planner for target lanelet dynamically
    if (!goal->lanelet_name.empty()) {
        end_lanelet_name_ = goal->lanelet_name;
        RCLCPP_INFO(this->get_logger(), "Action Server: Rebuilding global planner to lanelet '%s'", end_lanelet_name_.c_str());
        rebuildGlobalPlanner();
    }

    auto feedback = std::make_shared<NavigateToGoal::Feedback>();
    auto result = std::make_shared<NavigateToGoal::Result>();

    rclcpp::Rate rate(5.0); // 5Hz tracking rate
    double target_x = goal->target_pose.pose.position.x;
    double target_y = goal->target_pose.pose.position.y;

    // When the client sends only a lanelet_name and leaves target_pose at its
    // zero default, derive the target from the last global-plan waypoint so the
    // goal-reached check has a real position to work against.
    if (target_x == 0.0 && target_y == 0.0) {
        if (!all_waypoints_from_global_planner_.empty()) {
            const auto& last_wp = all_waypoints_from_global_planner_.back();
            target_x = last_wp.x;
            target_y = last_wp.y;
            RCLCPP_INFO(this->get_logger(),
                "Action Server: target_pose not set; using last plan waypoint (%.2f, %.2f)",
                target_x, target_y);
        } else {
            auto res = std::make_shared<NavigateToGoal::Result>();
            res->success = false;
            res->message = "No target_pose and no global plan waypoints available";
            goal_handle->abort(res);
            RCLCPP_ERROR(this->get_logger(), "Action Server: %s", res->message.c_str());
            return;
        }
    }
    double total_distance = 0.0;
    double start_time = this->now().seconds();
    double last_x, last_y;
    {
        std::lock_guard<std::mutex> lk(car_state_mutex_);
        last_x = car_state_->x;
        last_y = car_state_->y;
    }

    double tolerance = 0.3; // Default tolerance
    this->get_parameter("planner.safe_clear", tolerance); // use safe_clear or fallback

    while (rclcpp::ok() && goal_handle->is_active()) {
        if (goal_handle->is_canceling()) {
            std::lock_guard<std::mutex> lk(action_server_mutex_);
            if (goal_handle->is_active() || goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Goal canceled by client";
                result->total_distance = total_distance;
                result->total_time = this->now().seconds() - start_time;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Action Server: Goal cancelled");
            }
            return;
        }

        double rx, ry;
        {
            std::lock_guard<std::mutex> lk(car_state_mutex_);
            rx = car_state_->x;
            ry = car_state_->y;
        }

        double d_step = std::hypot(rx - last_x, ry - last_y);
        total_distance += d_step;
        last_x = rx;
        last_y = ry;

        double dist_remaining = std::hypot(rx - target_x, ry - target_y);

        feedback->distance_remaining = dist_remaining;
        feedback->estimated_time_remaining = dist_remaining / 0.3; // estimated at 0.3m/s speed
        feedback->current_pose.header.frame_id = "map";
        feedback->current_pose.header.stamp = this->now();
        feedback->current_pose.pose.position.x = rx;
        feedback->current_pose.pose.position.y = ry;
        feedback->current_pose.pose.position.z = 0.0;
        feedback->planner_status = "tracking";

        goal_handle->publish_feedback(feedback);

        // Check if goal reached
        if (dist_remaining <= tolerance) {
            std::lock_guard<std::mutex> lk(action_server_mutex_);
            if (!goal_handle->is_active()) {
                return;  // preempted by a newer goal between the distance check and here
            }
            RCLCPP_INFO(this->get_logger(), "Action Server: Goal reached! Remaining distance is %.2fm", dist_remaining);
            result->success = true;
            result->message = "Goal reached successfully";
            result->total_distance = total_distance;
            result->total_time = this->now().seconds() - start_time;
            goal_handle->succeed(result);
            return;
        }

        rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
