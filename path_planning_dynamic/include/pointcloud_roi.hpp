#ifndef POINTCLOUD_ROI_HPP
#define POINTCLOUD_ROI_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <string>

/**
 * @brief ROS 2 node that prepares raw point clouds for obstacle clustering.
 *
 * Rotates the incoming cloud, removes points that belong to the robot footprint,
 * crops the configured region of interest and optionally applies voxel filtering
 * plus ground removal before publishing the filtered cloud.
 *
 * @return --
 * @note Published topic names and filtering parameters are loaded from ROS parameters.
 */
class pointcloud_roi_node : public rclcpp::Node
{
private:
    // Ground-removal parameters.
    int num_lpr_;
    float th_seeds_;
    float th_dist_;
    float sensor_height_;

    /**
     * @brief Plane model used to separate ground from non-ground points.
     *
     * Stores the normal vector and offset of the estimated ground plane.
     *
     * @return --
     * @note The plane equation is normal.dot(point) + d = 0.
     */
    struct Model
    {
        Eigen::MatrixXf normal;
        double d = 0.;
    };

    /**
     * @brief Estimate a ground plane from seed points.
     *
     * Uses the mean and covariance matrix of the seed cloud, then takes the
     * smallest singular vector as the plane normal.
     *
     * @param seed_points Point cloud used to fit the plane.
     * @return Estimated plane model.
     * @note The input cloud should contain at least one valid seed point.
     */
    Model estimatePlane(const pcl::PointCloud<pcl::PointXYZ> &seed_points);

    /**
     * @brief Extract initial ground seed points.
     *
     * Filters candidate points by height, computes a low-point representative
     * height and keeps points close to that height as ground seeds.
     *
     * @param cloud_in Input point cloud used for seed extraction.
     * @param seed_points Output cloud populated with initial ground seeds.
     * @return --
     * @note If no valid low points exist, the output seed cloud is left empty.
     */
    void extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &seed_points);

    // Voxel-grid downsampling parameters.
    float voxel_leaf_size_x_ = 0.0;
    float voxel_leaf_size_y_ = 0.0;
    float voxel_leaf_size_z_ = 0.0;

    bool voxel_condition = false;

    // Region-of-interest bounds in the point-cloud frame.
    double roi_max_x_ = 0.0; // Front limit.
    double roi_max_y_ = 0.0; // Left limit.
    double roi_max_z_ = 0.0; // Upper limit.

    double roi_min_x_ = 0.0; // Rear limit.
    double roi_min_y_ = 0.0; // Right limit.
    double roi_min_z_ = 0.0; // Lower limit.

    // ROS topic and marker frame configuration.
    std::string pointcloud_topic = "none";
    std::string output_topic = "none";
    std::string output_topic_ground = "none";
    std::string name_space;
    std::string frame_id;
    std::string robot_footprint_topic;
    
    Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;

    // Sensor pitch correction applied before ROI filtering.
    float sensor_rotation_y_;
    Eigen::Matrix4f rotation_matrix_;

    // Robot body bounds removed from the incoming cloud.
    float robot_footprint_x_max;
    float robot_footprint_y_max;
    float robot_footprint_z_max;
    float robot_footprint_x_min;
    float robot_footprint_y_min;
    float robot_footprint_z_min;


    /**
     * @brief Process one incoming point-cloud message.
     *
     * Converts the cloud to PCL, applies rotation, removes robot footprint
     * points, crops the ROI and publishes the filtered outputs.
     *
     * @param msg Incoming ROS PointCloud2 message.
     * @return --
     * @note Empty point-cloud messages are ignored.
     */
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief Create an RViz marker for the robot footprint exclusion box.
     *
     * Builds a cube marker from the configured robot footprint min/max bounds.
     *
     * @return Visualization marker representing the robot footprint box.
     * @note The callback updates the marker header to match the incoming cloud.
     */
    visualization_msgs::msg::Marker createRobotFootprintMarker();

    /**
     * @brief Remove points inside the robot footprint box.
     *
     * Copies only points outside the configured robot body bounds into a new cloud.
     *
     * @param cloud Point cloud updated in place to contain only non-robot points.
     * @return --
     * @note Points inside the footprint are treated as self-observations and discarded.
     */
    void removeRobotFootprintPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    // ROS subscriber and publishers.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

public:
    /**
     * @brief Create the point-cloud ROI node.
     *
     * Declares and reads ROS parameters, configures publishers/subscriber and
     * prepares the ROI and rotation transforms used by the callback.
     *
     * @return --
     * @note The node starts processing after it is spun by rclcpp.
     */
    pointcloud_roi_node(/* args */);

    /**
     * @brief Destroy the point-cloud ROI node.
     *
     * Uses the default destructor because resources are owned by smart pointers.
     *
     * @return --
     * @note ROS entities are cleaned up by rclcpp object ownership.
     */
    ~pointcloud_roi_node() = default;
};

#endif // POINTCLOUD_ROI_HPP
