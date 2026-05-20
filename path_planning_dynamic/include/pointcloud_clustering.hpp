#ifndef POINTCLOUD_CLUSTERING_HPP
#define POINTCLOUD_CLUSTERING_HPP
// RORS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Custom msgs path_planning_dynamic for Obstacle and ObstacleCollection
#include "path_planning_dynamic/msg/obstacle_collection.hpp"

#include "obstacle_detector.hpp"

#include <memory>
#include <string>
#include <vector>

/**
 * @brief ROS 2 node that converts non-ground point clouds into obstacle polygons.
 *
 * Subscribes to the filtered point cloud, groups nearby points into clusters, builds
 * a convex or concave hull for each valid cluster, and publishes the resulting
 * ObstacleCollection for the planner.
 *
 * @return --
 * @note If perception data is invalid, the node only emits a red warning and does not publish fake or empty obstacles.
 */
class pointcloud_clustering_node : public rclcpp::Node
{
private:    
    double CLUSTER_THRESH;
    int CLUSTER_MAX_SIZE;
    int CLUSTER_MIN_SIZE;
    double CONCAVE_ALPHA;
    std::string FRAME_ID;
    std::string HULL_MODE;

    std::shared_ptr<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>> obstacle_detector;
    path_planning_dynamic::msg::ObstacleCollection obstacle_collection;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    /**
     * @brief Process incoming non-ground point clouds.
     *
     * Converts the ROS PointCloud2 message to PCL, runs Euclidean clustering,
     * and dispatches each cluster set to the configured hull generation mode.
     *
     * @param msg Incoming point cloud from the ROI / ground-removal node.
     * @return --
     * @note Invalid perception inputs only generate a warning; no obstacle message is published.
     */
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief Transform an incoming point cloud to the configured obstacle frame.
     *
     * Converts points from the sensor frame into FRAME_ID so hulls are generated
     * using rover x/y axes instead of camera x/y axes.
     *
     * @param msg Original ROS point-cloud message containing the source frame.
     * @param input_cloud PCL cloud decoded from msg.
     * @param output_cloud Output cloud in FRAME_ID.
     * @return true when the cloud is ready for clustering.
     */
    bool transformCloudToFrame(const sensor_msgs::msg::PointCloud2 &msg,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);

    /**
     * @brief Build and publish convex obstacle polygons from clustered points.
     *
     * Creates one closed polygon per valid cluster using PCL ConvexHull and
     * publishes the resulting ObstacleCollection when at least one valid obstacle exists.
     *
     * @param cloud_clusters Clustered point clouds returned by the obstacle detector.
     * @return --
     * @note Clusters that cannot produce a valid polygon are skipped.
     */
    void convex_hull(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_clusters);

    /**
     * @brief Build and publish concave obstacle polygons from clustered points.
     *
     * Creates one closed polygon per valid cluster using PCL ConcaveHull. If the
     * concave hull is invalid, it falls back to a convex hull for that cluster.
     *
     * @param cloud_clusters Clustered point clouds returned by the obstacle detector.
     * @return --
     * @note Publishes only when at least one valid obstacle polygon exists.
     */
    void concave_hull(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_clusters);

    /**
     * @brief Log unsafe or invalid perception states.
     *
     * Emits a red warning explaining why no obstacle update was published.
     *
     * @param warning_message Human-readable warning message.
     * @return --
     * @note This function intentionally does not publish /obstacle_info, /cmd_vel, or any service call.
     */
    void warnUnsafePerception(const std::string &warning_message);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    rclcpp::Publisher<path_planning_dynamic::msg::ObstacleCollection>::SharedPtr obstacle_info_publisher_;

public:
    /**
     * @brief Create the point cloud clustering node.
     *
     * Declares and reads clustering parameters, configures the point cloud
     * subscriber, obstacle publisher, and obstacle detector.
     *
     * @return --
     * @note The node publishes obstacle polygons on /obstacle_info only when valid obstacles are detected.
     */
    pointcloud_clustering_node();
    ~pointcloud_clustering_node() = default;
};

#endif // POINTCLOUD_CLUSTERING_HPP
