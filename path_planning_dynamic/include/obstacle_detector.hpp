#pragma once

#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <memory>
#include <vector>

namespace lidar_obstacle_detector
{
    /**
     * @brief Euclidean point-cloud cluster extractor for obstacle detection.
     *
     * Groups nearby points from a PCL point cloud into independent clusters that
     * can later be converted into obstacle polygons.
     *
     * @tparam PointT PCL point type used by the input cloud, such as pcl::PointXYZ.
     * @return --
     * @note The point type must be supported by PCL KdTree and EuclideanClusterExtraction.
     */
    template <typename PointT>
    class ObstacleDetector
    {
    public:
        /**
         * @brief Create an obstacle detector.
         *
         * Initializes the detector without owning any point-cloud data.
         *
         * @return --
         * @note All clustering parameters are provided per clustering() call.
         */
        ObstacleDetector() = default;

        /**
         * @brief Destroy the obstacle detector.
         *
         * Uses the default destructor because the detector does not own external resources.
         *
         * @return --
         * @note Point clouds created during clustering are returned through shared pointers.
         */
        virtual ~ObstacleDetector() = default;

        /**
         * @brief Split a point cloud into Euclidean clusters.
         *
         * Builds a KdTree for the input cloud, runs PCL EuclideanClusterExtraction
         * and copies each detected cluster into its own point cloud.
         *
         * @param cloud Input point cloud to cluster.
         * @param cluster_tolerance Maximum distance between neighboring points in the same cluster.
         * @param min_size Minimum number of points required for a valid cluster.
         * @param max_size Maximum number of points allowed for a valid cluster.
         * @return Vector of point-cloud clusters detected in the input cloud.
         * @note Returns an empty vector if the input cloud pointer is null.
         */
        std::vector<typename pcl::PointCloud<PointT>::Ptr>
        clustering(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const float cluster_tolerance, const int min_size, const int max_size);

    private:
    };

    template <typename PointT>
    std::vector<typename pcl::PointCloud<PointT>::Ptr>
    ObstacleDetector<PointT>::clustering(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const float cluster_tolerance, const int min_size, const int max_size)
    {
        if (!cloud) {
            return {};
        }


        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

        // Perform euclidean clustering to group detected obstacles
        typename pcl::search::KdTree<PointT>::Ptr tree =
            std::make_shared<pcl::search::KdTree<PointT>>();

        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_size);
        ec.setMaxClusterSize(max_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        clusters.reserve(cluster_indices.size());
        for (const auto &indices : cluster_indices)
        {
            typename pcl::PointCloud<PointT>::Ptr cluster =
                std::make_shared<pcl::PointCloud<PointT>>();
            cluster->points.reserve(indices.indices.size());
            for (int index : indices.indices)
                cluster->points.push_back(cloud->points[index]);

            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }

        return clusters;
    }

}
