// junction_gap_check — standalone diagnostic for Lanelet2 (.osm) maps used by
// the dynamic global planner.
//
// It answers one question: "Can the planner actually route between every
// lanelet, or do hand-drawn junctions fall outside the connection threshold?"
//
// It deliberately mirrors GlobalPlanner so its verdict matches real planner
// behaviour:
//   * same Origin / LocalCartesianProjector,
//   * the same local_x/local_y coordinate overwrite the planner applies,
//   * the same centerline endpoint distance metric,
//   * the same 1.5 m endpoint-connect threshold (kEndpointConnectDist),
//   * the same lateral relations (besides/left/right/adjacent) added as edges.
//
// Output:
//   * connected components of the reachability graph (lanelets in different
//     components cannot be routed between),
//   * "near-miss" endpoint pairs that sit just beyond the connect threshold and
//     bridge two different components — the likely-intended junctions to fix.
//
// Usage:
//   ros2 run path_planning_dynamic junction_gap_check [map_path]
//        [connect_threshold=1.5] [near_band=4.0] [x_offset=0] [y_offset=0]
//   map_path accepts a package:// URI or a filesystem path.

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
// Mirror GlobalPlanner's compile-time constant.
constexpr double kEndpointConnectDist = 1.5;

const std::string kGreen = "\033[1;32m";
const std::string kRed = "\033[1;31m";
const std::string kYellow = "\033[1;33m";
const std::string kBlue = "\033[1;34m";
const std::string kReset = "\033[0m";

struct LaneletGeo
{
    lanelet::Id id = 0;
    std::string name;
    lanelet::ConstPoint3d front;
    lanelet::ConstPoint3d back;
};

double dist2d(const lanelet::ConstPoint3d &a, const lanelet::ConstPoint3d &b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    return std::sqrt(dx * dx + dy * dy);
}

bool isCrosswalk(const lanelet::ConstLanelet &ll)
{
    return ll.hasAttribute(lanelet::AttributeName::Subtype) &&
           ll.attribute(lanelet::AttributeName::Subtype).value() ==
               lanelet::AttributeValueString::Crosswalk;
}

// Minimum distance between the closest pair of centerline endpoints — identical
// to GlobalPlanner::endpointDistance.
double endpointDistance(const LaneletGeo &a, const LaneletGeo &b)
{
    return std::min({dist2d(a.front, b.front), dist2d(a.front, b.back),
                     dist2d(a.back, b.front), dist2d(a.back, b.back)});
}

std::string resolvePackageUri(std::string path)
{
    if (path.rfind("package://", 0) != 0)
        return path;
    const std::string rest = path.substr(10);
    const auto slash = rest.find('/');
    if (slash == std::string::npos)
        return path;
    const std::string pkg = rest.substr(0, slash);
    const std::string rel = rest.substr(slash + 1);
    try
    {
        return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
    }
    catch (const std::exception &e)
    {
        std::cerr << kRed << "Could not resolve package '" << pkg << "': " << e.what()
                  << kReset << std::endl;
        return path;
    }
}

// Union-Find for connected components.
struct DisjointSet
{
    std::unordered_map<lanelet::Id, lanelet::Id> parent;

    lanelet::Id find(lanelet::Id x)
    {
        auto it = parent.find(x);
        if (it == parent.end())
            return parent[x] = x;
        if (it->second == x)
            return x;
        return it->second = find(it->second);
    }

    void unite(lanelet::Id a, lanelet::Id b)
    {
        parent[find(a)] = find(b);
    }
};
}  // namespace

int main(int argc, char **argv)
{
    std::string map_path = (argc > 1) ? argv[1] : "package://rover_bringup/maps/test1.osm";
    const double connect_threshold = (argc > 2) ? std::atof(argv[2]) : kEndpointConnectDist;
    const double near_band = (argc > 3) ? std::atof(argv[3]) : 4.0;
    const double x_offset = (argc > 4) ? std::atof(argv[4]) : 0.0;
    const double y_offset = (argc > 5) ? std::atof(argv[5]) : 0.0;

    map_path = resolvePackageUri(map_path);

    std::cout << kBlue << "junction_gap_check" << kReset << "\n"
              << "  map               : " << map_path << "\n"
              << "  connect_threshold : " << connect_threshold << " m\n"
              << "  near_band         : " << near_band << " m\n"
              << "  offsets           : (" << x_offset << ", " << y_offset << ")\n"
              << std::endl;

    lanelet::LaneletMapPtr map;
    try
    {
        lanelet::Origin origin({49, 8.4});
        lanelet::projection::LocalCartesianProjector projector(origin);
        map = lanelet::load(map_path, projector);
    }
    catch (const std::exception &e)
    {
        std::cerr << kRed << "Failed to load map: " << e.what() << kReset << std::endl;
        return 1;
    }
    if (!map)
    {
        std::cerr << kRed << "Map load returned null." << kReset << std::endl;
        return 1;
    }

    // Overwrite x/y with local_x/local_y + offset, exactly like the planner does,
    // so endpoint distances are computed in the same coordinate frame.
    for (auto &point : map->pointLayer)
    {
        if (!point.hasAttribute("local_x") || !point.hasAttribute("local_y"))
            continue;
        const auto lx = point.attribute("local_x").asDouble();
        const auto ly = point.attribute("local_y").asDouble();
        if (!lx || !ly)
            continue;
        point.x() = *lx + x_offset;
        point.y() = *ly + y_offset;
    }

    // Collect drivable lanelets with usable centerlines.
    std::vector<LaneletGeo> geos;
    for (const auto &ll : map->laneletLayer)
    {
        if (isCrosswalk(ll))
            continue;
        const auto centerline = ll.centerline3d();
        if (centerline.size() < 2)
            continue;
        LaneletGeo g;
        g.id = ll.id();
        if (ll.hasAttribute("name"))
            g.name = ll.attribute("name").value();
        g.front = centerline.front();
        g.back = centerline.back();
        geos.push_back(g);
    }

    if (geos.empty())
    {
        std::cerr << kRed << "No drivable lanelets with usable centerlines found." << kReset
                  << std::endl;
        return 1;
    }

    std::cout << "Drivable lanelets: " << geos.size() << "\n" << std::endl;

    // Build the reachability graph the same way GlobalPlanner does: endpoint
    // proximity edges + lateral relations from the routing graph.
    DisjointSet ds;
    for (const auto &g : geos)
        ds.find(g.id);

    std::size_t endpoint_edges = 0;
    for (std::size_t i = 0; i < geos.size(); ++i)
    {
        for (std::size_t j = i + 1; j < geos.size(); ++j)
        {
            if (endpointDistance(geos[i], geos[j]) <= connect_threshold)
            {
                ds.unite(geos[i].id, geos[j].id);
                ++endpoint_edges;
            }
        }
    }

    std::size_t lateral_edges = 0;
    try
    {
        auto trafficRules = lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle);
        auto routingGraph = lanelet::routing::RoutingGraph::build(*map, *trafficRules);
        if (routingGraph)
        {
            std::unordered_set<lanelet::Id> known;
            for (const auto &g : geos)
                known.insert(g.id);

            auto link = [&](const lanelet::ConstLanelet &a, const lanelet::ConstLanelet &b) {
                if (known.count(a.id()) && known.count(b.id()) && a.id() != b.id())
                {
                    ds.unite(a.id(), b.id());
                    ++lateral_edges;
                }
            };
            for (const auto &ll : map->laneletLayer)
            {
                if (!known.count(ll.id()))
                    continue;
                for (const auto &beside : routingGraph->besides(ll))
                    link(ll, beside);
                if (auto l = routingGraph->left(ll)) link(ll, *l);
                if (auto r = routingGraph->right(ll)) link(ll, *r);
                if (auto al = routingGraph->adjacentLeft(ll)) link(ll, *al);
                if (auto ar = routingGraph->adjacentRight(ll)) link(ll, *ar);
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << kYellow << "Routing graph build failed (" << e.what()
                  << "); reporting endpoint-only connectivity." << kReset << std::endl;
    }

    std::cout << "Edges: " << endpoint_edges << " endpoint, " << lateral_edges
              << " lateral\n" << std::endl;

    // Group lanelets into connected components.
    std::map<lanelet::Id, std::vector<LaneletGeo>> components;
    for (const auto &g : geos)
        components[ds.find(g.id)].push_back(g);

    std::cout << (components.size() == 1 ? kGreen : kRed)
              << "Connected components: " << components.size() << kReset << "\n";
    if (components.size() == 1)
    {
        std::cout << kGreen << "All lanelets are mutually reachable — no junction gaps."
                  << kReset << std::endl;
    }
    else
    {
        std::cout << kRed
                  << "Map is split into islands; the planner cannot route between them."
                  << kReset << std::endl;
    }

    int idx = 1;
    for (const auto &[root, members] : components)
    {
        std::cout << "\n  Component " << idx++ << " (" << members.size() << " lanelets): ";
        for (const auto &m : members)
        {
            std::cout << m.id;
            if (!m.name.empty())
                std::cout << "(" << m.name << ")";
            std::cout << " ";
        }
        std::cout << std::endl;
    }

    // Near-miss endpoint pairs: just beyond the connect threshold AND bridging two
    // different components. These are the junctions most likely meant to connect.
    if (components.size() > 1)
    {
        struct NearMiss { double gap; const LaneletGeo *a; const LaneletGeo *b; };
        std::vector<NearMiss> misses;
        for (std::size_t i = 0; i < geos.size(); ++i)
        {
            for (std::size_t j = i + 1; j < geos.size(); ++j)
            {
                if (ds.find(geos[i].id) == ds.find(geos[j].id))
                    continue;  // already in the same component
                const double gap = endpointDistance(geos[i], geos[j]);
                if (gap > connect_threshold && gap <= near_band)
                    misses.push_back({gap, &geos[i], &geos[j]});
            }
        }
        std::sort(misses.begin(), misses.end(),
                  [](const NearMiss &a, const NearMiss &b) { return a.gap < b.gap; });

        std::cout << "\n" << kYellow << "Near-miss junctions (gap in ("
                  << connect_threshold << ", " << near_band << "] m, different components): "
                  << misses.size() << kReset << std::endl;
        for (const auto &m : misses)
        {
            std::cout << "  " << kYellow << std::fixed
                      << "gap=" << m.gap << " m" << kReset
                      << "  lanelet " << m.a->id;
            if (!m.a->name.empty()) std::cout << "(" << m.a->name << ")";
            std::cout << "  <->  lanelet " << m.b->id;
            if (!m.b->name.empty()) std::cout << "(" << m.b->name << ")";
            std::cout << std::endl;
        }
        if (!misses.empty())
        {
            std::cout << kBlue
                      << "\nFix: move the touching endpoints of each pair to within "
                      << connect_threshold << " m (or raise the planner's "
                      << "kEndpointConnectDist)." << kReset << std::endl;
        }
    }

    return components.size() == 1 ? 0 : 2;
}
