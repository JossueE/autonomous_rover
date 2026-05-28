#ifndef GLOBAL_PLANNER_HPP
#define GLOBAL_PLANNER_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace lanelet;

struct point_struct { double x, y, heading;
                      int priority, lanelet_id, lane_sequence_id; };

class GlobalPlanner
{
private:
    // ANSI colors for terminal log output
    std::string green  = "\033[1;32m";
    std::string red    = "\033[1;31m";
    std::string blue   = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string reset  = "\033[0m";

    // Planner parameters
    double      waypoint_interval = 0.5;
    int         start_lanelet_id_ = 0;
    int         end_lanelet_id_   = 0;
    std::string start_lanelet_name_;
    std::string end_lanelet_name_;
    double      x_offset_         = 0.0;
    double      y_offset_         = 0.0;
    std::string map_path_;

    // Waypoint storage
    std::vector<std::vector<point_struct>> neighbor_points_;
    std::vector<point_struct>              all_waypoints_;

    // Cached per-route lookups (rebuilt at top of generateNeighborWaypoints)
    std::set<lanelet::Id> path_ids_;       // IDs of all lanelets in shortestPath
    std::set<lanelet::Id> reachable_ids_;  // IDs reachable (following/previous/adjacent) from shortestPath

    // Diagnostics
    bool verbose_logging_ = true;          // when false, suppresses per-iteration debug logs

    // Occupancy grid configuration & data
    double      resolution_     = 0.0;
    int         close_radius_   = 1;
    int         close_iters_    = 1;
    int         outside_value_  = 0;
    std::string frame_id_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    bool        occupancy_grid_ready_ = false;

    // ---------------------------------------------------------------
    // Tunables (kept identical to previously hard-coded magic numbers)
    // ---------------------------------------------------------------
    static constexpr double MAX_ROUTING_COST            = 500.0;

    // Priority codes (preserved meaning).
    static constexpr int    PRIORITY_MAIN               = 1;
    static constexpr int    PRIORITY_NEIGHBOR           = 2;
    static constexpr int    PRIORITY_BRANCHING          = 3;
    static constexpr int    PRIORITY_ADJACENT           = 4;

    // Trajectory compatibility.
    static constexpr double DIRECT_CONTINUATION_DIST    = 10.0;
    static constexpr double MAX_ANGLE_CONTINUATION_DEG  = 70.0;
    static constexpr double MAX_ANGLE_NORMAL_DEG        = 45.0;
    static constexpr double MAX_TRANSITION_ANGLE_DEG    = 90.0;
    static constexpr double MAX_AVG_DEV_CONTINUATION    = 50.0;
    static constexpr double MAX_AVG_DEV_NORMAL          = 35.0;
    static constexpr double CROSS_SUSPICIOUS            = 0.8;
    static constexpr double CROSS_EXTREME               = 1.2;
    static constexpr double DIVERGE_FACTOR_CONTINUATION = 1.0;
    static constexpr double DIVERGE_FACTOR_NORMAL       = 0.4;
    static constexpr int    TRAJECTORY_SAMPLES          = 5;
    static constexpr double ROUTE_LENGTH_MARGIN         = 1.5;

    // isBeyondTarget thresholds.
    static constexpr double DOT_SAME_DIRECTION          = 0.3;
    static constexpr double BEYOND_CONTINUATION_DIST    = 15.0;
    static constexpr double BEYOND_NEAR_DIST            = 5.0;

    // isBranchingLanelet thresholds.
    static constexpr double DOT_PARALLEL                = 0.9;
    static constexpr double PARALLEL_NEIGHBOR_DIST      = 10.0;

    // countMeaningfulConnections thresholds.
    static constexpr double START_CONNECTION_DIST       = 6.0;
    static constexpr double END_CONNECTION_DIST         = 5.0;
    static constexpr double MEANINGFUL_END_DIST         = 15.0;

    static constexpr double EPS                         = 1e-6;

    // ---------------------------------------------------------------
    // Routing & waypoint generation
    // ---------------------------------------------------------------
    void map_routing(lanelet::LaneletMapPtr &map);
    void generateNeighborWaypoints(lanelet::LaneletMapPtr &map,
                                   routing::RoutingGraphUPtr &routingGraph,
                                   const routing::LaneletPath &shortestPath);
    void addLaneletAsWaypoints(const lanelet::ConstLanelet &lanelet,
                               int priority, int lane_sequence_id);

    bool isBeyondTarget(const lanelet::ConstLanelet &lanelet,
                        const routing::LaneletPath &shortestPath);
    bool isBranchingLanelet(const lanelet::ConstLanelet &path_lanelet,
                            const lanelet::ConstLanelet &candidate_lanelet);
    bool isCompatibleTrajectory(const lanelet::ConstLanelet &path_lanelet,
                                const lanelet::ConstLanelet &candidate_lanelet,
                                routing::RoutingGraphUPtr &routingGraph,
                                const routing::LaneletPath &shortestPath,
                                lanelet::LaneletMapPtr &map);
    int  countMeaningfulConnections(const lanelet::ConstLanelet &candidate_lanelet,
                                    routing::RoutingGraphUPtr &routingGraph,
                                    const routing::LaneletPath &shortestPath,
                                    int current_path_index,
                                    lanelet::LaneletMapPtr &map);

    std::pair<double, double> getEndDirection  (const lanelet::ConstLineString3d &points);
    std::pair<double, double> getStartDirection(const lanelet::ConstLineString3d &points);

    double calculatePathLength(const routing::LaneletPath &path);
    double calculateRemainingPathLength(const routing::LaneletPath &path, int start_index);
    std::vector<point_struct> getAllWaypointsStruct() const;

    // Small numeric / lookup helpers
    static double               distance2d(const lanelet::ConstPoint3d &a, const lanelet::ConstPoint3d &b);
    static double               distance3d(const lanelet::ConstPoint3d &a, const lanelet::ConstPoint3d &b);
    static lanelet::ConstLanelets collectAdjacentPlus(routing::RoutingGraphUPtr &routingGraph,
                                                      const lanelet::ConstLanelet &lanelet,
                                                      bool include_besides);
    static int                  indexInShortestPath(const routing::LaneletPath &path, lanelet::Id id);
    bool                        isInMainPath(lanelet::Id id) const;  // uses path_ids_ cache
    bool                        resolveLaneletName(const lanelet::LaneletMapPtr &map,
                                                   const std::string &lanelet_name,
                                                   int &lanelet_id,
                                                   const std::string &label) const;

    // ---------------------------------------------------------------
    // Occupancy grid helpers
    // ---------------------------------------------------------------
    void generateOccupancyGrid(lanelet::LaneletMapPtr &t_map);
    void worldToGrid(double wx, double wy, double min_x, double min_y, int &gx, int &gy) const;
    void morphClose(std::vector<int8_t> &data, int width, int height, int radius, int iters) const;
    void fillLaneletPolygon(const std::vector<lanelet::ConstPoint3d> &points, int width, int height,
                            double min_x, double min_y, std::vector<int8_t> &grid, int8_t value) const;

public:
    GlobalPlanner(double x_offset, double y_offset, std::string map_path,
                  int start_lanelet_id, int end_lanelet_id,
                  std::string start_lanelet_name, std::string end_lanelet_name,
                  double resolution,
                  int close_radius, int close_iters, int outside_value,
                  std::string frame_id);

    std::vector<point_struct>     getAllAllWaypointsStruct();
    nav_msgs::msg::OccupancyGrid  getOccupancyGrid();
    bool                          isOccupancyGridReady();
};

#endif // GLOBAL_PLANNER_HPP
