#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP

#include <grid_map_core/grid_map_core.hpp>
#include <opencv2/opencv.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include "vehicle_footprint.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include "state.hpp"
#include <vector>
#include <iostream>
#include <tuple>

using namespace std;

/**
 * @brief Occupancy grid wrapper used for local collision checking.
 *
 * Converts a ROS OccupancyGrid into a grid_map with obstacle and distance
 * layers, then queries obstacle clearance for vehicle states during planning.
 *
 * @return --
 * @note The collision-check methods keep the existing behavior: true means a collision was detected.
 */
class Grid_map
{
private:
    nav_msgs::msg::OccupancyGrid map_data_;
    double resolution;
    double originX;
    double originY;
    unsigned int width;
    unsigned int height;
    double free_thres_ = 20;

    VehicleFootprint vehicle_footprint_;
    std::vector<Circle> footprint_buffer_;

    grid_map::GridMap map_;

public:
    /**
     * @brief Create a local grid map from an occupancy grid message.
     *
     * Stores map metadata, fills the obstacle layer and builds a distance field
     * used by collision checks.
     *
     * @param map_data Occupancy grid used as the source map.
     * @return --
     * @note Occupied and unknown cells are treated as obstacles.
     */
    Grid_map(const nav_msgs::msg::OccupancyGrid &map_data);

    /**
     * @brief Set the vehicle footprint used for collision checking.
     *
     * Copies the configured vehicle footprint and prepares the reusable circle
     * buffer used by detailed collision checks.
     *
     * @param vehicle_footprint Vehicle geometry approximation used by the planner.
     * @return --
     * @note Call this before checking vehicle states for collision.
     */
    void setVehicleFootprint(const VehicleFootprint &vehicle_footprint);

    /**
     * @brief Get obstacle clearance at a map position.
     *
     * Reads the distance field at the requested position using grid_map interpolation.
     *
     * @param pos Position in the map frame.
     * @return Distance to the nearest obstacle in meters, or 0.0 if outside the map.
     * @note This method performs its own bounds check.
     */
    double getObstacleDistance(const Eigen::Vector2d &pos) const;

    /**
     * @brief Check whether a position is inside the grid map.
     *
     * Uses the underlying grid_map geometry to validate the given map-frame position.
     *
     * @param pos Position in the map frame.
     * @return True if the position is inside the map bounds.
     * @note This only checks bounds, not obstacle occupancy.
     */
    bool isInside(const Eigen::Vector2d &pos) const;

    /**
     * @brief Check detailed vehicle footprint collision for one state.
     *
     * Transforms the vehicle footprint circles to the given state and compares
     * each circle radius against the distance field clearance.
     *
     * @param current Vehicle state to evaluate.
     * @return True if any footprint circle collides or goes outside the map.
     * @note False means the state is collision-free.
     */
    bool isSingleStateInCollision(const State &current);

    /**
     * @brief Check collision using a bounding-circle broad phase.
     *
     * Tests the vehicle bounding circle first, then runs the detailed footprint
     * check only when the bounding circle is near an obstacle.
     *
     * @param current Vehicle state to evaluate.
     * @return True if the state is in collision or outside the map.
     * @note False means the state is collision-free.
     */
    bool isSingleStateInCollisionImproved(const State &current);

    /**
     * @brief Convert a vehicle state position to occupancy-grid cell indices.
     *
     * Computes the grid cell from the state x/y coordinates, map origin and map
     * resolution.
     *
     * @param start_state State whose x/y position will be converted.
     * @return Tuple containing cell x and cell y.
     * @note The returned indices are not clamped to map bounds.
     */
    std::tuple<int, int> toCellID(const State& start_state);
};

#endif // GRID_MAP_HPP
