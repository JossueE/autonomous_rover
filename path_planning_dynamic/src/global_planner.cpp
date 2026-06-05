#include "global_planner.hpp"

#include <lanelet2_io/Io.h>
#include <rclcpp/clock.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace
{
constexpr double kEndpointConnectDist = 1.5;
constexpr double kLateralChangePenalty = 2.0;
constexpr double kLaneletNearestFallbackDist = 1.0;
constexpr double kPolygonBoundaryTolerance = 0.05;
constexpr double kMaxLateralHeadingDiff = 1.5707963267948966;
constexpr double kStartReverseHeadingPenalty = 5.0;

struct LaneletGeometry
{
    lanelet::ConstLanelet lanelet;
    lanelet::ConstLineString3d centerline;
    double length = 0.0;
};

struct OrientedLaneletState
{
    lanelet::Id id;
    bool reversed;

    bool operator==(const OrientedLaneletState &other) const noexcept
    {
        return id == other.id && reversed == other.reversed;
    }
};

struct OrientedLaneletStateHash
{
    std::size_t operator()(const OrientedLaneletState &state) const noexcept
    {
        const auto id_hash = std::hash<lanelet::Id>{}(state.id);
        const auto reverse_hash = std::hash<bool>{}(state.reversed);
        return id_hash ^ (reverse_hash + 0x9e3779b9 + (id_hash << 6) + (id_hash >> 2));
    }
};

struct GraphEdge
{
    OrientedLaneletState to;
    double cost;
};

using GeometryCache = std::unordered_map<lanelet::Id, LaneletGeometry>;
using GeometricGraph =
    std::unordered_map<OrientedLaneletState, std::vector<GraphEdge>, OrientedLaneletStateHash>;

double pointDistance2d(const lanelet::ConstPoint3d &a, const lanelet::ConstPoint3d &b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    return std::sqrt(dx * dx + dy * dy);
}

double pointDistance3d(const lanelet::ConstPoint3d &a, const lanelet::ConstPoint3d &b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    const double dz = a.z() - b.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double pointToSegmentDistance2d(double px, double py,
                                double ax, double ay,
                                double bx, double by)
{
    const double abx = bx - ax;
    const double aby = by - ay;
    const double apx = px - ax;
    const double apy = py - ay;
    const double len2 = abx * abx + aby * aby;

    if (len2 < 1e-12)
    {
        const double dx = px - ax;
        const double dy = py - ay;
        return std::sqrt(dx * dx + dy * dy);
    }

    const double t = std::max(0.0, std::min(1.0, (apx * abx + apy * aby) / len2));
    const double cx = ax + t * abx;
    const double cy = ay + t * aby;
    const double dx = px - cx;
    const double dy = py - cy;
    return std::sqrt(dx * dx + dy * dy);
}

double pointToCenterlineDistance2d(double x, double y, const lanelet::ConstLineString3d &points)
{
    if (points.empty())
        return std::numeric_limits<double>::infinity();
    if (points.size() == 1)
    {
        const double dx = x - points.front().x();
        const double dy = y - points.front().y();
        return std::sqrt(dx * dx + dy * dy);
    }

    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < points.size(); ++i)
    {
        best = std::min(best, pointToSegmentDistance2d(
            x, y,
            points[i - 1].x(), points[i - 1].y(),
            points[i].x(), points[i].y()));
    }
    return best;
}

bool pointInPolygon(double x, double y, const std::vector<std::pair<double, double>> &polygon)
{
    if (polygon.size() < 3)
        return false;

    bool inside = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
    {
        const auto &[xi, yi] = polygon[i];
        const auto &[xj, yj] = polygon[j];

        if (pointToSegmentDistance2d(x, y, xi, yi, xj, yj) <= kPolygonBoundaryTolerance)
            return true;

        const bool crosses = ((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (crosses)
            inside = !inside;
    }
    return inside;
}

std::vector<std::pair<double, double>> laneletPolygon2d(const lanelet::ConstLanelet &ll)
{
    std::vector<std::pair<double, double>> polygon;
    polygon.reserve(ll.leftBound().size() + ll.rightBound().size());

    for (const auto &p : ll.leftBound())
        polygon.emplace_back(p.x(), p.y());

    const auto &right_bound = ll.rightBound();
    for (int i = static_cast<int>(right_bound.size()) - 1; i >= 0; --i)
        polygon.emplace_back(right_bound[i].x(), right_bound[i].y());

    return polygon;
}

double lineStringLength(const lanelet::ConstLineString3d &points)
{
    double length = 0.0;
    for (size_t i = 1; i < points.size(); ++i)
        length += pointDistance3d(points[i], points[i - 1]);
    return length;
}

bool isCrosswalkLanelet(const lanelet::ConstLanelet &ll)
{
    return ll.hasAttribute(lanelet::AttributeName::Subtype) &&
           ll.attribute(lanelet::AttributeName::Subtype).value() ==
               lanelet::AttributeValueString::Crosswalk;
}

const lanelet::ConstPoint3d &stateStartPoint(const LaneletGeometry &geometry,
                                             bool reversed)
{
    return reversed ? geometry.centerline.back() : geometry.centerline.front();
}

const lanelet::ConstPoint3d &stateEndPoint(const LaneletGeometry &geometry,
                                           bool reversed)
{
    return reversed ? geometry.centerline.front() : geometry.centerline.back();
}

double orientedHeading(const LaneletGeometry &geometry, bool reversed)
{
    const auto &start = stateStartPoint(geometry, reversed);
    const auto &end = stateEndPoint(geometry, reversed);
    return std::atan2(end.y() - start.y(), end.x() - start.x());
}

double headingDifference(double a, double b)
{
    double diff = a - b;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff <= -M_PI) diff += 2.0 * M_PI;
    return std::abs(diff);
}

lanelet::ConstLanelet orientedLanelet(const GeometryCache &cache,
                                      const OrientedLaneletState &state)
{
    const auto it = cache.find(state.id);
    if (it == cache.end())
        throw std::out_of_range("oriented lanelet state missing from geometry cache");
    return state.reversed ? it->second.lanelet.invert() : it->second.lanelet;
}

void addDirectedEdge(GeometricGraph &graph,
                     const OrientedLaneletState &from,
                     const OrientedLaneletState &to,
                     double cost)
{
    if (from == to)
        return;

    auto &edges = graph[from];
    for (auto &edge : edges)
    {
        if (edge.to == to)
        {
            edge.cost = std::min(edge.cost, cost);
            return;
        }
    }
    edges.push_back({to, cost});
}

void addCompatibleLateralEdges(GeometricGraph &graph,
                               const GeometryCache &cache,
                               lanelet::Id a,
                               lanelet::Id b,
                               double penalty)
{
    const auto a_it = cache.find(a);
    const auto b_it = cache.find(b);
    if (a_it == cache.end() || b_it == cache.end())
        return;

    for (const bool a_reversed : {false, true})
    {
        for (const bool b_reversed : {false, true})
        {
            const double heading_a = orientedHeading(a_it->second, a_reversed);
            const double heading_b = orientedHeading(b_it->second, b_reversed);
            if (headingDifference(heading_a, heading_b) > kMaxLateralHeadingDiff)
                continue;

            const OrientedLaneletState state_a{a, a_reversed};
            const OrientedLaneletState state_b{b, b_reversed};
            addDirectedEdge(graph, state_a, state_b, b_it->second.length + penalty);
            addDirectedEdge(graph, state_b, state_a, a_it->second.length + penalty);
        }
    }
}

void appendUniqueLanelet(std::vector<lanelet::ConstLanelet> &out,
                         std::unordered_set<lanelet::Id> &seen,
                         const lanelet::ConstLanelet &candidate)
{
    if (seen.insert(candidate.id()).second)
        out.push_back(candidate);
}

std::vector<lanelet::ConstLanelet> collectLateralRelations(
    routing::RoutingGraphUPtr &routingGraph,
    const lanelet::ConstLanelet &lanelet)
{
    std::vector<lanelet::ConstLanelet> related;
    std::unordered_set<lanelet::Id> seen;

    auto collect_one = [&](const lanelet::ConstLanelet &query) {
        for (const auto &beside : routingGraph->besides(query))
            appendUniqueLanelet(related, seen, beside);
        if (auto l = routingGraph->left(query))          appendUniqueLanelet(related, seen, *l);
        if (auto r = routingGraph->right(query))         appendUniqueLanelet(related, seen, *r);
        if (auto al = routingGraph->adjacentLeft(query)) appendUniqueLanelet(related, seen, *al);
        if (auto ar = routingGraph->adjacentRight(query)) appendUniqueLanelet(related, seen, *ar);
    };

    collect_one(lanelet);
    collect_one(lanelet.invert());
    return related;
}

GeometryCache buildGeometryCache(const lanelet::LaneletMapPtr &map)
{
    GeometryCache cache;
    for (const auto &ll : map->laneletLayer)
    {
        if (isCrosswalkLanelet(ll))
            continue;

        const auto centerline = ll.centerline3d();
        if (centerline.size() < 2)
            continue;

        cache.emplace(ll.id(), LaneletGeometry{ll, centerline, lineStringLength(centerline)});
    }
    return cache;
}

GeometricGraph buildGeometricGraph(const GeometryCache &cache,
                                   routing::RoutingGraphUPtr &routingGraph)
{
    GeometricGraph graph;
    std::vector<OrientedLaneletState> states;
    states.reserve(cache.size() * 2);
    for (const auto &entry : cache)
    {
        const OrientedLaneletState forward{entry.first, false};
        const OrientedLaneletState reverse{entry.first, true};
        graph[forward];
        graph[reverse];
        states.push_back(forward);
        states.push_back(reverse);
    }

    for (size_t i = 0; i < states.size(); ++i)
    {
        const auto &a_state = states[i];
        const auto &a = cache.at(a_state.id);
        for (size_t j = 0; j < states.size(); ++j)
        {
            if (i == j) continue;
            const auto &b_state = states[j];
            if (a_state.id == b_state.id) continue;
            const auto &b = cache.at(b_state.id);
            if (pointDistance2d(stateEndPoint(a, a_state.reversed),
                                stateStartPoint(b, b_state.reversed)) <= kEndpointConnectDist)
            {
                addDirectedEdge(graph, a_state, b_state, b.length);
            }
        }
    }

    for (const auto &entry : cache)
    {
        for (const auto &related : collectLateralRelations(routingGraph, entry.second.lanelet))
        {
            if (cache.count(related.id()))
                addCompatibleLateralEdges(graph, cache, entry.first, related.id(),
                                          kLateralChangePenalty);
        }
    }

    return graph;
}

std::vector<OrientedLaneletState> dijkstraPath(
    const GeometricGraph &graph,
    const std::unordered_map<OrientedLaneletState, double, OrientedLaneletStateHash> &start_costs,
    const std::vector<OrientedLaneletState> &goal_states)
{
    if (start_costs.empty() || goal_states.empty())
        return {};

    struct QueueItem
    {
        double cost;
        OrientedLaneletState state;
    };
    struct QueueCompare
    {
        bool operator()(const QueueItem &a, const QueueItem &b) const
        {
            return a.cost > b.cost;
        }
    };

    auto is_goal = [&](const OrientedLaneletState &state) {
        return std::find(goal_states.begin(), goal_states.end(), state) != goal_states.end();
    };

    std::priority_queue<QueueItem, std::vector<QueueItem>, QueueCompare> open;
    std::unordered_map<OrientedLaneletState, double, OrientedLaneletStateHash> dist;
    std::unordered_map<OrientedLaneletState, OrientedLaneletState, OrientedLaneletStateHash> previous;

    for (const auto &entry : start_costs)
    {
        if (!graph.count(entry.first))
            continue;
        dist[entry.first] = entry.second;
        open.push({entry.second, entry.first});
    }

    OrientedLaneletState best_goal{0, false};
    bool have_goal = false;
    while (!open.empty())
    {
        const auto current = open.top();
        open.pop();

        const auto dist_it = dist.find(current.state);
        if (dist_it == dist.end() || current.cost > dist_it->second + 1e-9)
            continue;
        if (is_goal(current.state))
        {
            best_goal = current.state;
            have_goal = true;
            break;
        }

        const auto edge_it = graph.find(current.state);
        if (edge_it == graph.end())
            continue;

        for (const auto &edge : edge_it->second)
        {
            const double next_dist = current.cost + edge.cost;
            const auto next_it = dist.find(edge.to);
            if (next_it == dist.end() || next_dist < next_it->second)
            {
                dist[edge.to] = next_dist;
                previous[edge.to] = current.state;
                open.push({next_dist, edge.to});
            }
        }
    }

    if (!have_goal)
        return {};

    std::vector<OrientedLaneletState> states;
    for (auto state = best_goal;; state = previous[state])
    {
        states.push_back(state);
        if (!previous.count(state))
            break;
    }
    std::reverse(states.begin(), states.end());
    return states;
}

routing::LaneletPath laneletPathFromStates(const GeometryCache &cache,
                                           const std::vector<OrientedLaneletState> &states)
{
    lanelet::ConstLanelets lanelets;
    lanelets.reserve(states.size());
    for (const auto &state : states)
    {
        if (cache.count(state.id))
            lanelets.push_back(orientedLanelet(cache, state));
    }
    return routing::LaneletPath(std::move(lanelets));
}

double pathLengthFromStates(const GeometryCache &cache,
                            const std::vector<OrientedLaneletState> &states)
{
    double length = 0.0;
    for (const auto &state : states)
    {
        const auto it = cache.find(state.id);
        if (it != cache.end())
            length += it->second.length;
    }
    return length;
}
}  // namespace

// ===================================================================
// Constructor
// ===================================================================
GlobalPlanner::GlobalPlanner(double x_offset, double y_offset, std::string map_path,
                             int start_lanelet_id, int end_lanelet_id,
                             std::string start_lanelet_name, std::string end_lanelet_name,
                             double resolution,
                             int close_radius, int close_iters, int outside_value,
                             std::string frame_id,
                             bool has_start_heading,
                             double start_heading,
                             bool has_goal_pose,
                             double goal_x,
                             double goal_y)
    : start_lanelet_id_(start_lanelet_id),
      end_lanelet_id_(end_lanelet_id),
      start_lanelet_name_(std::move(start_lanelet_name)),
      end_lanelet_name_(std::move(end_lanelet_name)),
      x_offset_(x_offset),
      y_offset_(y_offset),
      map_path_(std::move(map_path)),
      has_start_heading_(has_start_heading),
      start_heading_(start_heading),
      has_goal_pose_(has_goal_pose),
      goal_x_(goal_x),
      goal_y_(goal_y),
      resolution_(resolution),
      close_radius_(close_radius),
      close_iters_(close_iters),
      outside_value_(outside_value),
      frame_id_(std::move(frame_id))
{
    // C5: defensive parameter validation.
    if (resolution_ <= 0.0)
    {
        std::cerr << red << "[GlobalPlanner] resolution must be > 0, got " << resolution_
                  << " - aborting initialization." << reset << std::endl;
        return;
    }

    // Resolve package:// URIs: "package://pkg/relative/path" → install share path
    if (map_path_.rfind("package://", 0) == 0)
    {
        std::string rest = map_path_.substr(10);
        auto slash = rest.find('/');
        if (slash != std::string::npos)
        {
            std::string pkg = rest.substr(0, slash);
            std::string rel = rest.substr(slash + 1);
            try
            {
                map_path_ = ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
            }
            catch (const std::exception &e)
            {
                std::cerr << red << "[GlobalPlanner] Error resolving package share directory for " 
                          << pkg << ": " << e.what() << reset << std::endl;
            }
        }
    }

    // C1: protect against map-load exceptions (file missing, malformed, etc.).
    lanelet::LaneletMapPtr map;
    try
    {
        lanelet::Origin origin({49, 8.4});
        lanelet::projection::LocalCartesianProjector projector(origin);
        map = lanelet::load(map_path_, projector);
    }
    catch (const std::exception &e)
    {
        std::cerr << red << "[GlobalPlanner] Failed to load map '" << map_path_
                  << "': " << e.what() << reset << std::endl;
        return;
    }
    if (!map)
    {
        std::cerr << red << "[GlobalPlanner] Map load returned null." << reset << std::endl;
        return;
    }

    // C2: validate local_x / local_y attributes before applying offsets.
    size_t skipped_points = 0;
    for (auto &point : map->pointLayer)
    {
        if (!point.hasAttribute("local_x") || !point.hasAttribute("local_y"))
        {
            ++skipped_points;
            continue;
        }
        const auto lx = point.attribute("local_x").asDouble();
        const auto ly = point.attribute("local_y").asDouble();
        if (!lx || !ly)
        {
            ++skipped_points;
            continue;
        }
        point.x() = *lx + x_offset_;
        point.y() = *ly + y_offset_;
    }
    if (skipped_points > 0)
    {
        std::cerr << yellow << "[GlobalPlanner] Skipped " << skipped_points
                  << " point(s) lacking local_x/local_y." << reset << std::endl;
    }
    map_ = map;

    if (!start_lanelet_name_.empty() &&
        !resolveLaneletName(map, start_lanelet_name_, start_lanelet_id_, "start_lanelet_name"))
    {
        return;
    }
    if (!end_lanelet_name_.empty() &&
        !resolveLaneletName(map, end_lanelet_name_, end_lanelet_id_, "end_lanelet_name"))
    {
        return;
    }

    map_routing(map);
    generateOccupancyGrid(map);
    // C7: do NOT unconditionally set occupancy_grid_ready_ = true here;
    // generateOccupancyGrid sets it to true only when generation succeeds.
}

// ===================================================================
// Numeric / lookup helpers
// ===================================================================
double GlobalPlanner::distance2d(const lanelet::ConstPoint3d &a, const lanelet::ConstPoint3d &b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    return std::sqrt(dx * dx + dy * dy);
}

double GlobalPlanner::distance3d(const lanelet::ConstPoint3d &a, const lanelet::ConstPoint3d &b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    const double dz = a.z() - b.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// D2: gather all "neighbor-ish" lanelets in one place
// (besides + left + right + adjacentLeft + adjacentRight).
lanelet::ConstLanelets GlobalPlanner::collectAdjacentPlus(routing::RoutingGraphUPtr &routingGraph,
                                                          const lanelet::ConstLanelet &lanelet,
                                                          bool include_besides)
{
    lanelet::ConstLanelets out;
    if (include_besides)
        out = routingGraph->besides(lanelet);
    if (auto l  = routingGraph->left(lanelet))           out.push_back(*l);
    if (auto r  = routingGraph->right(lanelet))          out.push_back(*r);
    if (auto al = routingGraph->adjacentLeft(lanelet))   out.push_back(*al);
    if (auto ar = routingGraph->adjacentRight(lanelet))  out.push_back(*ar);
    return out;
}

// D5: linear scan but in a named function for clarity.
int GlobalPlanner::indexInShortestPath(const routing::LaneletPath &path, lanelet::Id id)
{
    for (size_t i = 0; i < path.size(); ++i)
    {
        if (path[i].id() == id)
            return static_cast<int>(i);
    }
    return -1;
}

bool GlobalPlanner::isInMainPath(lanelet::Id id) const
{
    return path_ids_.count(id) > 0;
}

bool GlobalPlanner::resolveLaneletName(const lanelet::LaneletMapPtr &map,
                                       const std::string &lanelet_name,
                                       int &lanelet_id,
                                       const std::string &label) const
{
    for (const auto &ll : map->laneletLayer)
    {
        if (ll.hasAttribute("name") &&
            ll.attribute("name").value() == lanelet_name)
        {
            lanelet_id = static_cast<int>(ll.id());
            std::cout << green << "[GlobalPlanner] " << label << "='" << lanelet_name
                      << "' resolved to lanelet_id=" << lanelet_id << reset << std::endl;
            return true;
        }
    }

    std::cerr << red << "[GlobalPlanner] Could not find lanelet with name='"
              << lanelet_name << "' for " << label << reset << std::endl;
    return false;
}

// ===================================================================
// Routing graph build & route discovery
// ===================================================================
void GlobalPlanner::map_routing(lanelet::LaneletMapPtr &map)
{
    auto trafficRules = traffic_rules::TrafficRulesFactory::create(
        Locations::Germany, Participants::Vehicle);
    auto routingGraph = routing::RoutingGraph::build(*map, *trafficRules);

    if (!routingGraph)
        return;

    std::cout << green << "Routing graph built successfully" << reset << std::endl;

    // C3: validate that start/end IDs exist before calling get() (which throws).
    const auto startIt = map->laneletLayer.find(start_lanelet_id_);
    const auto endIt   = map->laneletLayer.find(end_lanelet_id_);
    if (startIt == map->laneletLayer.end() || endIt == map->laneletLayer.end())
    {
        std::cerr << red << "[GlobalPlanner] start_lanelet_id=" << start_lanelet_id_
                  << " or end_lanelet_id=" << end_lanelet_id_
                  << " not found in laneletLayer." << reset << std::endl;
        return;
    }
    const auto startLanelet = *startIt;
    const auto endLanelet   = *endIt;

    const auto geometry_cache = buildGeometryCache(map);
    if (!geometry_cache.count(startLanelet.id()) || !geometry_cache.count(endLanelet.id()))
    {
        std::cerr << red << "[GlobalPlanner] start/end lanelet missing usable centerline geometry."
                  << reset << std::endl;
        return;
    }

    const auto geometric_graph = buildGeometricGraph(geometry_cache, routingGraph);
    std::unordered_map<OrientedLaneletState, double, OrientedLaneletStateHash> start_costs;
    for (const bool reversed : {false, true})
    {
        const OrientedLaneletState state{startLanelet.id(), reversed};
        double cost = 0.0;
        if (has_start_heading_)
        {
            const double route_heading = orientedHeading(geometry_cache.at(startLanelet.id()), reversed);
            if (headingDifference(route_heading, start_heading_) > kMaxLateralHeadingDiff)
                cost += kStartReverseHeadingPenalty;
        }
        start_costs[state] = cost;
    }

    std::vector<OrientedLaneletState> goal_states = {
        {endLanelet.id(), false},
        {endLanelet.id(), true}
    };
    if (has_goal_pose_)
    {
        std::vector<OrientedLaneletState> selected_goal_states;
        double best_goal_distance = std::numeric_limits<double>::infinity();
        for (const auto &state : goal_states)
        {
            const auto &end_point = stateEndPoint(geometry_cache.at(state.id), state.reversed);
            const double dx = end_point.x() - goal_x_;
            const double dy = end_point.y() - goal_y_;
            const double distance = std::sqrt(dx * dx + dy * dy);
            if (distance + EPS < best_goal_distance)
            {
                best_goal_distance = distance;
                selected_goal_states.clear();
                selected_goal_states.push_back(state);
            }
            else if (std::abs(distance - best_goal_distance) <= EPS)
            {
                selected_goal_states.push_back(state);
            }
        }
        if (!selected_goal_states.empty())
        {
            goal_states = std::move(selected_goal_states);
            std::cout << yellow << "[GlobalPlanner] Goal endpoint selected from target_pose ("
                      << goal_x_ << ", " << goal_y_ << "), distance="
                      << best_goal_distance << " m, reversed="
                      << (goal_states.front().reversed ? "true" : "false")
                      << reset << std::endl;
        }
    }

    const auto oriented_path = dijkstraPath(geometric_graph, start_costs, goal_states);

    if (oriented_path.empty())
    {
        std::cout << red << "Goal lanelet is not reachable from the start lanelet in the bidirectional oriented graph." << reset << std::endl;
        return;
    }

    const auto shortest_path = laneletPathFromStates(geometry_cache, oriented_path);
    const double path_length = pathLengthFromStates(geometry_cache, oriented_path);
    std::cout << green << "Bidirectional oriented route found, length=" << path_length
              << " m, lanelets=" << shortest_path.size() << reset << std::endl;
    for (const auto &state : oriented_path)
    {
        if (state.reversed)
        {
            std::cout << yellow << "[GlobalPlanner] Using lanelet " << state.id
                      << " in reverse orientation." << reset << std::endl;
        }
    }
    generateNeighborWaypoints(map, routingGraph, shortest_path);
}

// ===================================================================
// Single-lanelet -> waypoints converter
// ===================================================================
void GlobalPlanner::addLaneletAsWaypoints(const lanelet::ConstLanelet &lanelet,
                                          int priority, int lane_sequence_id)
{
    // C5: defensive guard (waypoint_interval is hardcoded > 0 but stay safe).
    if (waypoint_interval <= 0.0)
        return;

    const auto centerline = lanelet.centerline3d();
    if (centerline.empty())
        return;

    struct WaypointSample
    {
        double x;
        double y;
        double z;
    };

    std::vector<WaypointSample> waypoints;
    waypoints.push_back({centerline.front().x(), centerline.front().y(), centerline.front().z()});

    double accumulated_distance = 0.0;
    double next_sample_distance = waypoint_interval;
    for (size_t i = 1; i < centerline.size(); ++i)
    {
        const auto &a = centerline[i - 1];
        const auto &b = centerline[i];
        const double segment_length = distance3d(b, a);
        if (segment_length < EPS)
            continue;

        while (next_sample_distance <= accumulated_distance + segment_length)
        {
            const double t = (next_sample_distance - accumulated_distance) / segment_length;
            waypoints.push_back({
                a.x() + t * (b.x() - a.x()),
                a.y() + t * (b.y() - a.y()),
                a.z() + t * (b.z() - a.z())
            });
            next_sample_distance += waypoint_interval;
        }

        accumulated_distance += segment_length;
    }

    const auto &last_centerline_point = centerline.back();
    const auto &last_waypoint = waypoints.back();
    const double dx = last_waypoint.x - last_centerline_point.x();
    const double dy = last_waypoint.y - last_centerline_point.y();
    const double dz = last_waypoint.z - last_centerline_point.z();
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > EPS)
    {
        waypoints.push_back({
            last_centerline_point.x(),
            last_centerline_point.y(),
            last_centerline_point.z()
        });
    }

    std::vector<point_struct> out;
    out.reserve(waypoints.size());
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        const auto &p = waypoints[i];
        double yaw = 0.0;
        if (i + 1 < waypoints.size())
        {
            const auto &n = waypoints[i + 1];
            yaw = std::atan2(n.y - p.y, n.x - p.x);
        }
        else if (i > 0)
        {
            const auto &q = waypoints[i - 1];
            yaw = std::atan2(p.y - q.y, p.x - q.x);
        }
        out.push_back({ p.x, p.y, yaw, priority,
                        static_cast<int>(lanelet.id()), lane_sequence_id });
    }
    neighbor_points_.push_back(std::move(out));
}

// ===================================================================
// Waypoint generation across main, neighbor, branching and adjacent
// ===================================================================
void GlobalPlanner::generateNeighborWaypoints(lanelet::LaneletMapPtr &map,
                                              routing::RoutingGraphUPtr &routingGraph,
                                              const routing::LaneletPath &shortestPath)
{
    std::cout << green << "Generating neighbor waypoints for routing path..." << reset << std::endl;

    std::set<lanelet::Id> processed_lanelets;
    neighbor_points_.clear();

    auto append_unique = [](lanelet::ConstLanelets &out,
                            std::unordered_set<lanelet::Id> &seen,
                            const lanelet::ConstLanelet &candidate) {
        if (seen.insert(candidate.id()).second)
            out.push_back(candidate);
    };

    auto collect_besides_bidirectional = [&](const lanelet::ConstLanelet &base) {
        lanelet::ConstLanelets out;
        std::unordered_set<lanelet::Id> seen;
        auto collect_one = [&](const lanelet::ConstLanelet &query) {
            for (const auto &ll : routingGraph->besides(query))
                append_unique(out, seen, ll);
        };
        collect_one(base);
        collect_one(base.invert());
        return out;
    };

    auto collect_adjacent_bidirectional = [&](const lanelet::ConstLanelet &base) {
        lanelet::ConstLanelets out;
        std::unordered_set<lanelet::Id> seen;
        auto collect_one = [&](const lanelet::ConstLanelet &query) {
            for (const auto &ll : collectAdjacentPlus(routingGraph, query, /*include_besides=*/false))
                append_unique(out, seen, ll);
        };
        collect_one(base);
        collect_one(base.invert());
        return out;
    };

    auto collect_following_bidirectional = [&](const lanelet::ConstLanelet &base) {
        lanelet::ConstLanelets out;
        std::unordered_set<lanelet::Id> seen;
        auto collect_one = [&](const lanelet::ConstLanelet &query) {
            for (const auto &ll : routingGraph->following(query, true))
                append_unique(out, seen, ll);
        };
        collect_one(base);
        collect_one(base.invert());
        return out;
    };

    auto collect_follow_previous_bidirectional = [&](const lanelet::ConstLanelet &base) {
        lanelet::ConstLanelets out;
        std::unordered_set<lanelet::Id> seen;
        auto collect_one = [&](const lanelet::ConstLanelet &query) {
            for (const auto &ll : routingGraph->following(query, true))
                append_unique(out, seen, ll);
            for (const auto &ll : routingGraph->previous(query, true))
                append_unique(out, seen, ll);
        };
        collect_one(base);
        collect_one(base.invert());
        return out;
    };

    // D1: build the path-ids cache for O(1) main-path membership.
    path_ids_.clear();
    for (const auto &ll : shortestPath)
        path_ids_.insert(ll.id());

    // D4: build the reachable-ids cache once per route.
    reachable_ids_.clear();
    for (const auto &main_ll : shortestPath)
    {
        for (const auto &x : collect_follow_previous_bidirectional(main_ll)) reachable_ids_.insert(x.id());
        for (const auto &x : collect_besides_bidirectional(main_ll))         reachable_ids_.insert(x.id());
        for (const auto &x : collect_adjacent_bidirectional(main_ll))        reachable_ids_.insert(x.id());
    }

    // (1) Main routing path -> priority 1, lane_sequence_id = 0
    size_t main_path_groups = 0;
    for (const auto &path_lanelet : shortestPath)
    {
        if (!processed_lanelets.insert(path_lanelet.id()).second)
            continue;
        addLaneletAsWaypoints(path_lanelet, PRIORITY_MAIN, 0);
        ++main_path_groups;
    }

    // (2) Neighboring lanelets grouped by subsequential connectivity -> priority 2
    std::vector<lanelet::ConstLanelet> all_neighbor_lanelets;
    for (const auto &path_lanelet : shortestPath)
    {
        for (const auto &lanelet : collect_besides_bidirectional(path_lanelet))
        {
            if (lanelet.id() == path_lanelet.id())
                continue;
            if (processed_lanelets.count(lanelet.id()))
                continue;
            all_neighbor_lanelets.push_back(lanelet);
        }
    }

    std::set<lanelet::Id> grouped_lanelets;
    std::vector<std::vector<lanelet::ConstLanelet>> neighbor_groups;

    for (const auto &seed : all_neighbor_lanelets)
    {
        if (grouped_lanelets.count(seed.id()))
            continue;

        std::vector<lanelet::ConstLanelet> current_group;
        std::queue<lanelet::ConstLanelet> to_process;
        to_process.push(seed);

        auto enqueue_if_neighbor = [&](const lanelet::ConstLanelet &ll) {
            if (grouped_lanelets.count(ll.id()))
                return;
            if (std::find(all_neighbor_lanelets.begin(), all_neighbor_lanelets.end(), ll)
                != all_neighbor_lanelets.end())
            {
                to_process.push(ll);
            }
        };

        while (!to_process.empty())
        {
            auto current_lanelet = to_process.front();
            to_process.pop();

            if (!grouped_lanelets.insert(current_lanelet.id()).second)
                continue;

            current_group.push_back(current_lanelet);

            for (const auto &ll : collect_follow_previous_bidirectional(current_lanelet))
                enqueue_if_neighbor(ll);
        }

        if (!current_group.empty())
            neighbor_groups.push_back(std::move(current_group));
    }

    int lane_sequence_id = 1;
    size_t neighbor_group_count = neighbor_groups.size();
    for (const auto &group : neighbor_groups)
    {
        for (const auto &lanelet : group)
        {
            processed_lanelets.insert(lanelet.id());
            addLaneletAsWaypoints(lanelet, PRIORITY_NEIGHBOR, lane_sequence_id);
        }
        lane_sequence_id++;
    }

    // (3) Branching lanelets -> priority 3
    std::cout << blue << "Finding lanelets that branch off through curves..." << reset << std::endl;
    size_t branching_count = 0;
    for (const auto &path_lanelet : shortestPath)
    {
        const auto following_lanelets = collect_following_bidirectional(path_lanelet);
        if (verbose_logging_)
        {
            std::cout << yellow << "Path lanelet " << path_lanelet.id() << " has "
                      << following_lanelets.size() << " bidirectional following lanelets" << reset << std::endl;
        }

        for (const auto &following_lanelet : following_lanelets)
        {
            if (processed_lanelets.count(following_lanelet.id()))
                continue;
            if (isInMainPath(following_lanelet.id()))   // D1
                continue;

            if (verbose_logging_)
            {
                std::cout << blue << "  Evaluating following lanelet " << following_lanelet.id()
                          << " for trajectory compatibility..." << reset << std::endl;
            }

            if (!isCompatibleTrajectory(path_lanelet, following_lanelet, routingGraph, shortestPath, map))
            {
                if (verbose_logging_)
                    std::cout << yellow << "  Following lanelet " << following_lanelet.id()
                              << " filtered out due to incompatible trajectory" << reset << std::endl;
                continue;
            }
            if (!isBranchingLanelet(path_lanelet, following_lanelet))
            {
                if (verbose_logging_)
                    std::cout << yellow << "  Following lanelet " << following_lanelet.id()
                              << " filtered out - not a branching lanelet" << reset << std::endl;
                continue;
            }
            if (isBeyondTarget(following_lanelet, shortestPath))
            {
                if (verbose_logging_)
                    std::cout << yellow << "  Following lanelet " << following_lanelet.id()
                              << " filtered out due to being beyond target" << reset << std::endl;
                continue;
            }

            if (verbose_logging_)
                std::cout << green << "  Adding following lanelet " << following_lanelet.id()
                          << " as purple arrow" << reset << std::endl;

            processed_lanelets.insert(following_lanelet.id());
            addLaneletAsWaypoints(following_lanelet, PRIORITY_BRANCHING, lane_sequence_id);
            lane_sequence_id++;
            ++branching_count;
        }
    }

    // (4) Adjacent lanelets -> priority 4
    std::cout << blue << "Finding lanelets connected through adjacency..." << reset << std::endl;
    size_t adjacent_count = 0;
    for (const auto &path_lanelet : shortestPath)
    {
        const auto adjacent_lanelets = collect_adjacent_bidirectional(path_lanelet);

        if (verbose_logging_)
        {
            std::cout << yellow << "Path lanelet " << path_lanelet.id() << " has "
                      << adjacent_lanelets.size() << " adjacent lanelets" << reset << std::endl;
        }

        for (const auto &adjacent_lanelet : adjacent_lanelets)
        {
            if (processed_lanelets.count(adjacent_lanelet.id()))
                continue;
            if (isInMainPath(adjacent_lanelet.id()))   // D1
                continue;

            if (verbose_logging_)
                std::cout << blue << "  Evaluating adjacent lanelet " << adjacent_lanelet.id()
                          << " as direction-agnostic adjacency..." << reset << std::endl;

            if (isBeyondTarget(adjacent_lanelet, shortestPath))
            {
                if (verbose_logging_)
                    std::cout << yellow << "  Adjacent lanelet " << adjacent_lanelet.id()
                              << " filtered out due to being beyond target" << reset << std::endl;
                continue;
            }

            if (verbose_logging_)
                std::cout << green << "  Adding adjacent lanelet " << adjacent_lanelet.id()
                          << " as purple arrow" << reset << std::endl;

            processed_lanelets.insert(adjacent_lanelet.id());
            addLaneletAsWaypoints(adjacent_lanelet, PRIORITY_ADJACENT, lane_sequence_id);
            lane_sequence_id++;
            ++adjacent_count;
        }
    }

    all_waypoints_ = getAllWaypointsStruct();

    // C8: accurate, informative summary log.
    std::cout << green << "Routing summary:"
              << " main_path_groups=" << main_path_groups
              << ", neighbor_groups="  << neighbor_group_count
              << ", branching_lanelets=" << branching_count
              << ", adjacent_lanelets=" << adjacent_count
              << ", total_waypoint_groups=" << neighbor_points_.size()
              << ", total_waypoints=" << all_waypoints_.size()
              << reset << std::endl;
}

std::vector<point_struct> GlobalPlanner::getAllAllWaypointsStruct()
{
    return all_waypoints_;
}

bool GlobalPlanner::findLaneletAt(double x, double y, int &lanelet_id, bool &inside) const
{
    lanelet_id = 0;
    inside = false;

    if (!map_)
        return false;

    double best_distance = std::numeric_limits<double>::infinity();
    lanelet::Id best_id = 0;
    double best_inside_distance = std::numeric_limits<double>::infinity();
    lanelet::Id best_inside_id = 0;
    int inside_candidates = 0;

    for (const auto &ll : map_->laneletLayer)
    {
        if (isCrosswalkLanelet(ll))
            continue;

        const auto centerline = ll.centerline3d();
        if (centerline.empty())
            continue;

        const double distance = pointToCenterlineDistance2d(x, y, centerline);
        if (pointInPolygon(x, y, laneletPolygon2d(ll)))
        {
            ++inside_candidates;
            if (distance < best_inside_distance)
            {
                best_inside_distance = distance;
                best_inside_id = ll.id();
            }
        }

        if (distance < best_distance)
        {
            best_distance = distance;
            best_id = ll.id();
        }
    }

    if (best_inside_id != 0)
    {
        lanelet_id = static_cast<int>(best_inside_id);
        inside = true;
        if (inside_candidates > 1 && ambiguous_lanelet_logs_ < 20)
        {
            ++ambiguous_lanelet_logs_;
            std::cout << yellow << "[GlobalPlanner] Pose (" << x << ", " << y
                      << ") is inside " << inside_candidates
                      << " overlapping lanelets; selected lanelet_id=" << lanelet_id
                      << " by nearest centerline distance=" << best_inside_distance
                      << " m." << reset << std::endl;
        }
        return true;
    }

    if (best_id != 0 && best_distance <= kLaneletNearestFallbackDist)
    {
        lanelet_id = static_cast<int>(best_id);
        inside = false;
        return true;
    }

    return false;
}

// Public name->id resolver over the loaded map. Mirrors the private overload
// used in the constructor but works against the already-stored map_, so callers
// (e.g. the action server) can resolve a goal name before rebuilding.
bool GlobalPlanner::resolveLaneletName(const std::string &lanelet_name, int &lanelet_id) const
{
    if (!map_ || lanelet_name.empty())
        return false;

    for (const auto &ll : map_->laneletLayer)
    {
        if (ll.hasAttribute("name") &&
            ll.attribute("name").value() == lanelet_name)
        {
            lanelet_id = static_cast<int>(ll.id());
            return true;
        }
    }
    return false;
}

// ===================================================================
// Geometry-based filtering
// ===================================================================
bool GlobalPlanner::isBeyondTarget(const lanelet::ConstLanelet &lanelet,
                                   const routing::LaneletPath &shortestPath)
{
    if (shortestPath.empty())
        return false;

    const auto &target_lanelet = shortestPath.back();
    const auto lanelet_points  = lanelet.centerline3d();
    const auto target_points   = target_lanelet.centerline3d();

    if (lanelet_points.empty() || target_points.empty())
        return false;

    const auto target_end    = target_points.back();
    const auto lanelet_start = lanelet_points.front();

    const double dx = lanelet_start.x() - target_end.x();
    const double dy = lanelet_start.y() - target_end.y();
    const double distance = std::sqrt(dx * dx + dy * dy);

    // C4: guard against division by zero (NaN propagation).
    // When the candidate lanelet starts exactly at the target's end,
    // it is a continuation -> treat as beyond the target.
    if (distance < EPS)
        return true;

    double target_dx = target_end.x() - target_points.front().x();
    double target_dy = target_end.y() - target_points.front().y();
    const double target_length = std::sqrt(target_dx * target_dx + target_dy * target_dy);

    if (target_length > EPS)
    {
        target_dx /= target_length;
        target_dy /= target_length;
        const double dot_product = (dx / distance) * target_dx + (dy / distance) * target_dy;

        if (dot_product > DOT_SAME_DIRECTION)
        {
            // Same direction as target; if close, treat as continuation beyond target.
            if (distance < BEYOND_CONTINUATION_DIST)
                return true;
        }
        else if (dot_product < -DOT_SAME_DIRECTION)
        {
            // Opposite direction; this is before the target.
            return false;
        }
    }
    // Perpendicular / unclear: only filter very close lanelets.
    return distance < BEYOND_NEAR_DIST;
}

bool GlobalPlanner::isBranchingLanelet(const lanelet::ConstLanelet &path_lanelet,
                                       const lanelet::ConstLanelet &candidate_lanelet)
{
    const auto path_points      = path_lanelet.centerline3d();
    const auto candidate_points = candidate_lanelet.centerline3d();

    if (path_points.size() < 2 || candidate_points.size() < 2)
        return false;

    double path_dx = path_points.back().x() - path_points.front().x();
    double path_dy = path_points.back().y() - path_points.front().y();
    const double path_length = std::sqrt(path_dx * path_dx + path_dy * path_dy);
    if (path_length < EPS)
        return false;
    path_dx /= path_length;
    path_dy /= path_length;

    double candidate_dx = candidate_points.back().x() - candidate_points.front().x();
    double candidate_dy = candidate_points.back().y() - candidate_points.front().y();
    const double candidate_length = std::sqrt(candidate_dx * candidate_dx + candidate_dy * candidate_dy);
    if (candidate_length < EPS)
        return false;
    candidate_dx /= candidate_length;
    candidate_dy /= candidate_length;

    const double dot_product   = path_dx * candidate_dx + path_dy * candidate_dy;
    const double start_distance = distance2d(candidate_points.front(), path_points.front());

    // Almost parallel + close together => parallel neighbor, not a branch.
    if (dot_product > DOT_PARALLEL && start_distance < PARALLEL_NEIGHBOR_DIST)
        return false;

    // Branching range or sharp turn: treated as a branch.
    return true;
}

bool GlobalPlanner::isCompatibleTrajectory(const lanelet::ConstLanelet &path_lanelet,
                                           const lanelet::ConstLanelet &candidate_lanelet,
                                           routing::RoutingGraphUPtr &routingGraph,
                                           const routing::LaneletPath &shortestPath,
                                           lanelet::LaneletMapPtr &map)
{
    const auto path_points      = path_lanelet.centerline3d();
    const auto candidate_points = candidate_lanelet.centerline3d();

    if (path_points.size() < 3 || candidate_points.size() < 3)
        return false;

    // (1) Connection quality between path end and candidate start.
    const double connection_distance = distance2d(candidate_points.front(), path_points.back());
    const bool is_direct_continuation = connection_distance < DIRECT_CONTINUATION_DIST;

    // (2) Overall direction compatibility.
    double path_dx = path_points.back().x() - path_points.front().x();
    double path_dy = path_points.back().y() - path_points.front().y();
    const double path_length = std::sqrt(path_dx * path_dx + path_dy * path_dy);

    double candidate_dx = candidate_points.back().x() - candidate_points.front().x();
    double candidate_dy = candidate_points.back().y() - candidate_points.front().y();
    const double candidate_length = std::sqrt(candidate_dx * candidate_dx + candidate_dy * candidate_dy);

    if (path_length < EPS || candidate_length < EPS)
        return false;

    path_dx /= path_length;
    path_dy /= path_length;
    candidate_dx /= candidate_length;
    candidate_dy /= candidate_length;

    const double overall_dot   = path_dx * candidate_dx + path_dy * candidate_dy;
    const double overall_angle = std::acos(std::max(-1.0, std::min(1.0, overall_dot))) * 180.0 / M_PI;
    const double max_allowed_angle = is_direct_continuation
                                     ? MAX_ANGLE_CONTINUATION_DEG
                                     : MAX_ANGLE_NORMAL_DEG;

    if (overall_angle > max_allowed_angle)
    {
        if (verbose_logging_)
            std::cout << yellow << "    Trajectory rejected: poor overall direction alignment" << reset << std::endl;
        return false;
    }

    // (3) Transition smoothness at the connection point.
    const auto path_end_dir   = getEndDirection(path_points);
    const auto cand_start_dir = getStartDirection(candidate_points);

    const double transition_dot   = path_end_dir.first  * cand_start_dir.first +
                                    path_end_dir.second * cand_start_dir.second;
    const double transition_angle = std::acos(std::max(-1.0, std::min(1.0, transition_dot))) * 180.0 / M_PI;

    if (verbose_logging_)
        std::cout << yellow << "    Transition angle: " << transition_angle << "°" << reset << std::endl;

    if (transition_angle > MAX_TRANSITION_ANGLE_DEG)
    {
        if (verbose_logging_)
            std::cout << yellow << "    Trajectory rejected: sharp transition at connection" << reset << std::endl;
        return false;
    }

    // (4) Curve-aware progressive deviation analysis.
    const int num_samples = std::min<int>(TRAJECTORY_SAMPLES,
        static_cast<int>(std::min(path_points.size(), candidate_points.size())) - 1);

    double cumulative_deviation = 0.0;
    double cumulative_cross     = 0.0;
    int    valid_samples        = 0;
    // C6: cross_sign_changes was computed but never used; removed.

    for (int i = 0; i < num_samples; ++i)
    {
        const int path_idx = (i * (static_cast<int>(path_points.size())      - 1)) / num_samples;
        const int cand_idx = (i * (static_cast<int>(candidate_points.size()) - 1)) / num_samples;

        if (path_idx + 1 >= static_cast<int>(path_points.size()) ||
            cand_idx + 1 >= static_cast<int>(candidate_points.size()))
            continue;

        double pdx = path_points[path_idx + 1].x() - path_points[path_idx].x();
        double pdy = path_points[path_idx + 1].y() - path_points[path_idx].y();
        const double plen = std::sqrt(pdx * pdx + pdy * pdy);

        double cdx = candidate_points[cand_idx + 1].x() - candidate_points[cand_idx].x();
        double cdy = candidate_points[cand_idx + 1].y() - candidate_points[cand_idx].y();
        const double clen = std::sqrt(cdx * cdx + cdy * cdy);

        if (plen < EPS || clen < EPS)
            continue;

        pdx /= plen; pdy /= plen;
        cdx /= clen; cdy /= clen;

        const double seg_dot = pdx * cdx + pdy * cdy;
        cumulative_deviation += std::acos(std::max(-1.0, std::min(1.0, seg_dot)));

        const double seg_cross = pdx * cdy - pdy * cdx;
        cumulative_cross += seg_cross;
        valid_samples++;
    }

    if (valid_samples == 0)
        return false;

    const double avg_deviation_deg = (cumulative_deviation / valid_samples) * 180.0 / M_PI;
    const double max_avg_deviation = is_direct_continuation
                                     ? MAX_AVG_DEV_CONTINUATION
                                     : MAX_AVG_DEV_NORMAL;
    if (avg_deviation_deg > max_avg_deviation)
    {
        if (verbose_logging_)
            std::cout << yellow << "    Trajectory rejected: excessive average deviation" << reset << std::endl;
        return false;
    }

    // (5) Connectivity check.
    const double avg_cross = cumulative_cross / valid_samples;
    if (verbose_logging_)
        std::cout << yellow << "    Turn analysis: avg_cross=" << avg_cross << reset << std::endl;

    const int current_path_index = indexInShortestPath(shortestPath, path_lanelet.id());   // D5

    const int meaningful_connections = countMeaningfulConnections(
        candidate_lanelet, routingGraph, shortestPath, current_path_index, map);
    if (verbose_logging_)
        std::cout << yellow << "    Connectivity analysis: meaningful_connections="
                  << meaningful_connections << reset << std::endl;

    if (meaningful_connections < 2)
    {
        if (verbose_logging_)
            std::cout << yellow << "    Trajectory rejected: insufficient meaningful connections ("
                      << meaningful_connections
                      << " < 2). Valid curves/paths must connect to at least 2 different lanelets or destinations."
                      << reset << std::endl;
        return false;
    }
    if (std::abs(avg_cross) > CROSS_SUSPICIOUS && meaningful_connections < 2)
    {
        if (verbose_logging_)
            std::cout << yellow << "    Trajectory rejected: very strong divergent turning with very limited connections"
                      << reset << std::endl;
        return false;
    }
    if (std::abs(avg_cross) > CROSS_EXTREME)
    {
        if (verbose_logging_)
            std::cout << yellow << "    Trajectory rejected: extremely sharp turning" << reset << std::endl;
        return false;
    }

    // (6) Divergence check.
    const double start_distance = distance2d(candidate_points.front(), path_points.front());
    const double end_distance   = distance2d(candidate_points.back(),  path_points.back());

    const double max_divergence_factor = is_direct_continuation
                                         ? DIVERGE_FACTOR_CONTINUATION
                                         : DIVERGE_FACTOR_NORMAL;
    const double max_lanelet_length = std::max(path_length, candidate_length);

    if (end_distance > start_distance + max_lanelet_length * max_divergence_factor)
    {
        if (verbose_logging_)
            std::cout << yellow << "    Trajectory rejected: excessive divergence" << reset << std::endl;
        return false;
    }

    if (verbose_logging_)
        std::cout << green << "    Trajectory accepted: compatible path (continuation="
                  << (is_direct_continuation ? "YES" : "NO") << ")" << reset << std::endl;
    return true;
}

// ===================================================================
// Meaningful connection counter (used by isCompatibleTrajectory)
// ===================================================================
int GlobalPlanner::countMeaningfulConnections(const lanelet::ConstLanelet &candidate_lanelet,
                                              routing::RoutingGraphUPtr &routingGraph,
                                              const routing::LaneletPath &shortestPath,
                                              int current_path_index,
                                              lanelet::LaneletMapPtr &map)
{
    int meaningful_connections = 0;
    int start_connections      = 0;
    int end_connections        = 0;

    const auto candidate_following = routingGraph->following(candidate_lanelet, true);
    const auto candidate_previous  = routingGraph->previous(candidate_lanelet, true);

    // D2: candidate's adjacent lanelets (without besides for this case).
    const auto adjacent_lanelets = collectAdjacentPlus(routingGraph, candidate_lanelet, /*include_besides=*/false);

    const auto candidate_points = candidate_lanelet.centerline3d();
    if (candidate_points.empty())
        return 0;

    const auto candidate_start = candidate_points.front();
    const auto candidate_end   = candidate_points.back();

    // (a) Reachability to final destination.
    try
    {
        auto route_to_destination = routingGraph->getRoute(candidate_lanelet, shortestPath.back(), 0);
        if (route_to_destination)
        {
            const double candidate_route_length    = calculatePathLength(route_to_destination->shortestPath());
            const double main_path_remaining_length = calculateRemainingPathLength(shortestPath, current_path_index);
            if (candidate_route_length <= main_path_remaining_length * ROUTE_LENGTH_MARGIN)
                meaningful_connections++;
        }
    }
    catch (...) {}

    // (b) Connection to path-ahead via following/previous.
    bool connects_to_path_ahead = false;
    if (current_path_index >= 0)
    {
        auto try_connect_ahead = [&](const lanelet::ConstLanelets &candidates) {
            for (const auto &c : candidates)
            {
                for (size_t i = static_cast<size_t>(current_path_index) + 1; i < shortestPath.size(); ++i)
                {
                    if (c.id() == shortestPath[i].id())
                    {
                        connects_to_path_ahead = true;
                        meaningful_connections++;
                        return;
                    }
                }
            }
        };
        try_connect_ahead(candidate_following);
        if (!connects_to_path_ahead)
            try_connect_ahead(candidate_previous);
    }

    // Mark already-counted lanelets so we do not double-count below.
    std::set<lanelet::Id> counted_lanelets;
    if (meaningful_connections > 0)
    {
        try
        {
            auto route_to_destination = routingGraph->getRoute(candidate_lanelet, shortestPath.back(), 0);
            if (route_to_destination)
            {
                for (const auto &path_ll : route_to_destination->shortestPath())
                    counted_lanelets.insert(path_ll.id());
            }
        }
        catch (...) {}

        if (current_path_index >= 0)
        {
            for (size_t i = static_cast<size_t>(current_path_index) + 1; i < shortestPath.size(); ++i)
                counted_lanelets.insert(shortestPath[i].id());
        }
    }

    // (c) Unique following/previous lanelets.
    auto count_unique = [&](const lanelet::ConstLanelets &lanelets) {
        for (const auto &ll : lanelets)
        {
            if (counted_lanelets.insert(ll.id()).second)
                meaningful_connections++;
        }
    };
    count_unique(candidate_following);
    count_unique(candidate_previous);

    // (d) Candidate's adjacent lanelets (main-path matches weight 2).
    for (const auto &adj : adjacent_lanelets)
    {
        if (counted_lanelets.count(adj.id()))
            continue;

        const bool in_main_path = isInMainPath(adj.id());   // D1
        if (in_main_path)
        {
            meaningful_connections += 2;
            if (verbose_logging_)
                std::cout << yellow << "      Adjacent lanelet " << adj.id()
                          << " is part of main path - counted as 2 connections" << reset << std::endl;
        }
        else
        {
            meaningful_connections += 1;
            if (verbose_logging_)
                std::cout << yellow << "      Adjacent lanelet " << adj.id()
                          << " counted as 1 connection" << reset << std::endl;
        }
        counted_lanelets.insert(adj.id());
    }

    // Endpoint-based connection check applied to main, reachable, and neighbor lanelets.
    auto count_endpoint_connections = [&](const lanelet::ConstLanelet &other,
                                          const char *label,
                                          bool log_end_distances)
    {
        if (counted_lanelets.count(other.id()))
            return;

        const auto pts = other.centerline3d();
        if (pts.empty())
            return;

        const auto s = pts.front();
        const auto e = pts.back();

        const double ds_s = distance2d(candidate_start, s);
        const double ds_e = distance2d(candidate_start, e);
        const double de_s = distance2d(candidate_end,   s);
        const double de_e = distance2d(candidate_end,   e);

        if (ds_s < START_CONNECTION_DIST || ds_e < START_CONNECTION_DIST)
        {
            start_connections++;
            meaningful_connections++;
            counted_lanelets.insert(other.id());
            if (verbose_logging_)
                std::cout << yellow << "      " << label << " lanelet " << other.id()
                          << " connected at START (dist=" << std::min(ds_s, ds_e) << "m)"
                          << reset << std::endl;
        }

        if (log_end_distances && verbose_logging_)
        {
            std::cout << yellow << "      " << label << " lanelet " << other.id()
                      << " END distances: to_start=" << de_s
                      << "m, to_end=" << de_e << "m" << reset << std::endl;
        }

        if (de_s < END_CONNECTION_DIST || de_e < END_CONNECTION_DIST)
        {
            end_connections++;
            if (counted_lanelets.insert(other.id()).second)
                meaningful_connections++;
            if (verbose_logging_)
                std::cout << yellow << "      " << label << " lanelet " << other.id()
                          << " connected at END (dist=" << std::min(de_s, de_e) << "m)"
                          << reset << std::endl;
        }
    };

    // (e) Main path endpoint connections.
    for (const auto &main_ll : shortestPath)
        count_endpoint_connections(main_ll, "Main path", true);

    // (f) Reachable lanelets endpoint connections (D4: use cached reachable_ids_).
    for (const auto reachable_id : reachable_ids_)
    {
        if (counted_lanelets.count(reachable_id))
            continue;
        if (reachable_id == candidate_lanelet.id())
        {
            if (verbose_logging_)
                std::cout << yellow << "      Skipping self-connection for lanelet "
                          << reachable_id << reset << std::endl;
            continue;
        }

        // C9: protect against unexpected missing IDs in laneletLayer.
        try
        {
            count_endpoint_connections(map->laneletLayer.get(reachable_id), "Reachable", false);
        }
        catch (...)
        {
            if (verbose_logging_)
                std::cout << yellow << "      Reachable lanelet " << reachable_id
                          << " not in laneletLayer - skipped" << reset << std::endl;
        }
    }

    // (g) Neighbor lanelets (besides + adjacent of each path lanelet).
    for (const auto &path_lanelet : shortestPath)
    {
        const auto lane_lanelets = collectAdjacentPlus(routingGraph, path_lanelet, /*include_besides=*/true);  // D2
        for (const auto &neighbor_ll : lane_lanelets)
            count_endpoint_connections(neighbor_ll, "Neighbor", true);
    }

    if (meaningful_connections < 1)
    {
        if (verbose_logging_)
            std::cout << yellow << "    Lanelet rejected: no meaningful connections found" << reset << std::endl;
        return 0;
    }
    if (start_connections == 0)
    {
        if (verbose_logging_)
            std::cout << yellow << "    Lanelet rejected: no connections at START - curves must connect to blue/orange paths at beginning"
                      << reset << std::endl;
        return 0;
    }
    if (end_connections == 0)
    {
        if (verbose_logging_)
            std::cout << yellow << "    Lanelet rejected: no connections at END - curves must connect to blue/orange paths at end"
                      << reset << std::endl;
        return 0;
    }

    // Meaningful END connections (only to main path / neighbor lanelets, within MEANINGFUL_END_DIST).
    int meaningful_end_connections = 0;
    auto add_if_meaningful_end = [&](const lanelet::ConstLanelet &other) {
        const auto pts = other.centerline3d();
        if (pts.empty())
            return;
        const double de_s = distance2d(candidate_end, pts.front());
        const double de_e = distance2d(candidate_end, pts.back());
        if (de_s < MEANINGFUL_END_DIST || de_e < MEANINGFUL_END_DIST)
            meaningful_end_connections++;
    };

    for (const auto &main_ll : shortestPath)
        add_if_meaningful_end(main_ll);

    for (const auto &path_lanelet : shortestPath)
    {
        const auto lane_lanelets = collectAdjacentPlus(routingGraph, path_lanelet, /*include_besides=*/true);  // D2
        for (const auto &neighbor_ll : lane_lanelets)
            add_if_meaningful_end(neighbor_ll);
    }

    if (verbose_logging_)
        std::cout << yellow << "    End connection analysis: total_end_connections=" << end_connections
                  << ", meaningful_end_connections=" << meaningful_end_connections << reset << std::endl;

    if (meaningful_end_connections == 0)
    {
        if (verbose_logging_)
            std::cout << yellow << "    Lanelet rejected: no meaningful end connections - curves must connect to main path or neighbor lanelets at end"
                      << reset << std::endl;
        return 0;
    }

    if (verbose_logging_)
        std::cout << yellow << "    Lanelet accepted: has meaningful connections at both start and end (start="
                  << start_connections << ", meaningful_end=" << meaningful_end_connections
                  << ", total=" << meaningful_connections << ")" << reset << std::endl;

    return meaningful_connections;
}

// ===================================================================
// Direction helpers
// ===================================================================
std::pair<double, double> GlobalPlanner::getEndDirection(const lanelet::ConstLineString3d &points)
{
    if (points.size() < 2)
        return {0, 0};

    const int start_idx = std::max(0, static_cast<int>(points.size()) - 3);
    const int end_idx   = static_cast<int>(points.size()) - 1;

    const double dx = points[end_idx].x() - points[start_idx].x();
    const double dy = points[end_idx].y() - points[start_idx].y();
    const double length = std::sqrt(dx * dx + dy * dy);
    if (length < EPS)
        return {0, 0};

    return {dx / length, dy / length};
}

std::pair<double, double> GlobalPlanner::getStartDirection(const lanelet::ConstLineString3d &points)
{
    if (points.size() < 2)
        return {0, 0};

    const int start_idx = 0;
    const int end_idx   = std::min(2, static_cast<int>(points.size()) - 1);

    const double dx = points[end_idx].x() - points[start_idx].x();
    const double dy = points[end_idx].y() - points[start_idx].y();
    const double length = std::sqrt(dx * dx + dy * dy);
    if (length < EPS)
        return {0, 0};

    return {dx / length, dy / length};
}

// ===================================================================
// Path length helpers
// ===================================================================
double GlobalPlanner::calculateRemainingPathLength(const routing::LaneletPath &path, int start_index)
{
    if (start_index < 0 || start_index >= static_cast<int>(path.size()))
        return 0.0;

    double remaining_length = 0.0;
    for (size_t i = static_cast<size_t>(start_index); i < path.size(); ++i)
    {
        const auto points = path[i].centerline3d();
        for (size_t j = 1; j < points.size(); ++j)
            remaining_length += distance3d(points[j], points[j - 1]);
    }
    return remaining_length;
}

double GlobalPlanner::calculatePathLength(const routing::LaneletPath &path)
{
    return calculateRemainingPathLength(path, 0);
}

std::vector<point_struct> GlobalPlanner::getAllWaypointsStruct() const
{
    size_t total = 0;
    for (const auto &lp : neighbor_points_)
        total += lp.size();

    std::vector<point_struct> filtered_points;
    filtered_points.reserve(total);
    for (const auto &lp : neighbor_points_)
        filtered_points.insert(filtered_points.end(), lp.begin(), lp.end());
    return filtered_points;
}

// ===================================================================
// Occupancy grid generation
// ===================================================================
void GlobalPlanner::generateOccupancyGrid(lanelet::LaneletMapPtr &t_map)
{
    std::cout << "Generating occupancy grid from lanelets..." << std::endl;

    // C5: defensive parameter validation.
    if (resolution_ <= 0.0)
    {
        std::cerr << red << "[GlobalPlanner] resolution_ <= 0 in generateOccupancyGrid; skipping."
                  << reset << std::endl;
        return;
    }

    // 1) Compute bounding box from all lanelet boundary points (excluding crosswalks).
    double min_x =  std::numeric_limits<double>::infinity();
    double min_y =  std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    auto updateBounds = [&](double x, double y) {
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    };

    auto isCrosswalk = [](const lanelet::ConstLanelet &ll) {
        return ll.hasAttribute(lanelet::AttributeName::Subtype) &&
               ll.attribute(lanelet::AttributeName::Subtype).value() ==
                   lanelet::AttributeValueString::Crosswalk;
    };

    for (const auto &ll : t_map->laneletLayer)
    {
        if (isCrosswalk(ll))
            continue;
        for (const auto &p : ll.leftBound())  updateBounds(p.x(), p.y());
        for (const auto &p : ll.rightBound()) updateBounds(p.x(), p.y());
    }

    if (!std::isfinite(min_x) || !std::isfinite(min_y) ||
        !std::isfinite(max_x) || !std::isfinite(max_y))
    {
        std::cout << "Invalid bounds computed; skipping occupancy grid generation." << std::endl;
        return;
    }

    // 2) Calculate grid dimensions.
    int width  = static_cast<int>(std::ceil((max_x - min_x) / resolution_)) + 1;
    int height = static_cast<int>(std::ceil((max_y - min_y) / resolution_)) + 1;
    width  = std::max(1, width);
    height = std::max(1, height);

    std::cout << "Grid dimensions: " << width << "x" << height
              << ", resolution: " << resolution_ << std::endl;

    // 3) Initialize grid with the outside-value (occupied/unknown).
    std::vector<int8_t> grid(static_cast<size_t>(width) * static_cast<size_t>(height),
                             static_cast<int8_t>(outside_value_));

    // 4) Fill lanelet polygons with free-space (value 0).
    for (const auto &ll : t_map->laneletLayer)
    {
        if (isCrosswalk(ll))
            continue;

        std::vector<lanelet::ConstPoint3d> polygon_points;
        for (const auto &p : ll.leftBound())
            polygon_points.push_back(p);

        const auto &right_bound = ll.rightBound();
        for (int i = static_cast<int>(right_bound.size()) - 1; i >= 0; --i)
            polygon_points.push_back(right_bound[i]);

        if (!polygon_points.empty())
            polygon_points.push_back(polygon_points[0]);

        fillLaneletPolygon(polygon_points, width, height, min_x, min_y, grid, 0);
    }

    // 5) Apply morphological closing to seal gaps.
    if (close_radius_ > 0 && close_iters_ > 0)
        morphClose(grid, width, height, close_radius_, close_iters_);

    // 6) Fill occupancy grid message.
    occupancy_grid_.header.stamp           = rclcpp::Clock().now();
    occupancy_grid_.header.frame_id        = frame_id_;
    occupancy_grid_.info.map_load_time     = occupancy_grid_.header.stamp;
    occupancy_grid_.info.resolution        = static_cast<float>(resolution_);
    occupancy_grid_.info.width             = static_cast<uint32_t>(width);
    occupancy_grid_.info.height            = static_cast<uint32_t>(height);
    occupancy_grid_.info.origin.position.x = min_x;
    occupancy_grid_.info.origin.position.y = min_y;
    occupancy_grid_.info.origin.position.z = 0.0;
    occupancy_grid_.info.origin.orientation.w = 1.0;

    occupancy_grid_.data = std::move(grid);
    occupancy_grid_ready_ = true;

    std::cout << "Occupancy grid generated successfully!" << std::endl;
}

void GlobalPlanner::worldToGrid(double wx, double wy, double min_x, double min_y, int &gx, int &gy) const
{
    gx = static_cast<int>(std::floor((wx - min_x) / resolution_));
    gy = static_cast<int>(std::floor((wy - min_y) / resolution_));
}

void GlobalPlanner::morphClose(std::vector<int8_t> &data, int width, int height,
                               int radius, int iters) const
{
    if (radius <= 0 || iters <= 0)
        return;

    auto dilate = [&](std::vector<int8_t> &src) {
        std::vector<int8_t> dst = src;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (src[y * width + x] == 0) {
                    for (int j = -radius; j <= radius; ++j) {
                        for (int i = -radius; i <= radius; ++i) {
                            const int nx = x + i, ny = y + j;
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                                dst[ny * width + nx] = 0;
                        }
                    }
                }
            }
        }
        src.swap(dst);
    };

    auto erode = [&](std::vector<int8_t> &src) {
        std::vector<int8_t> dst = src;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (src[y * width + x] == 0) {
                    bool keep = true;
                    for (int j = -radius; j <= radius && keep; ++j) {
                        for (int i = -radius; i <= radius; ++i) {
                            const int nx = x + i, ny = y + j;
                            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                            if (src[ny * width + nx] != 0) keep = false;
                        }
                    }
                    if (!keep) dst[y * width + x] = static_cast<int8_t>(outside_value_);
                }
            }
        }
        src.swap(dst);
    };

    for (int k = 0; k < iters; ++k) { dilate(data); erode(data); }
}

void GlobalPlanner::fillLaneletPolygon(const std::vector<lanelet::ConstPoint3d> &points,
                                       int width, int height,
                                       double min_x, double min_y,
                                       std::vector<int8_t> &grid, int8_t value) const
{
    if (points.size() < 3)
        return;

    std::vector<std::pair<int, int>> grid_points;
    grid_points.reserve(points.size());
    for (const auto &point : points)
    {
        int gx, gy;
        worldToGrid(point.x(), point.y(), min_x, min_y, gx, gy);
        grid_points.push_back({gx, gy});
    }

    int min_y_grid = height, max_y_grid = 0;
    for (const auto &p : grid_points)
    {
        min_y_grid = std::min(min_y_grid, p.second);
        max_y_grid = std::max(max_y_grid, p.second);
    }

    for (int y = min_y_grid; y <= max_y_grid; ++y)
    {
        std::vector<int> intersections;

        for (size_t i = 0; i + 1 < grid_points.size(); ++i)
        {
            const int y1 = grid_points[i].second;
            const int y2 = grid_points[i + 1].second;
            if ((y1 <= y && y < y2) || (y2 <= y && y < y1))
            {
                const int x1 = grid_points[i].first;
                const int x2 = grid_points[i + 1].first;
                const int x = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
                intersections.push_back(x);
            }
        }

        std::sort(intersections.begin(), intersections.end());
        for (size_t i = 0; i + 1 < intersections.size(); i += 2)
        {
            for (int x = intersections[i]; x <= intersections[i + 1]; ++x)
            {
                if (x >= 0 && x < width && y >= 0 && y < height)
                    grid[y * width + x] = value;
            }
        }
    }
}

nav_msgs::msg::OccupancyGrid GlobalPlanner::getOccupancyGrid()
{
    return occupancy_grid_;
}

bool GlobalPlanner::isOccupancyGridReady()
{
    return occupancy_grid_ready_;
}
