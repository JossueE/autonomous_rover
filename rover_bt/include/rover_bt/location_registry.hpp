#pragma once

#include <string>
#include <vector>
#include <map>
#include <optional>
#include <mutex>

namespace rover_bt {

/**
 * @brief Thread-safe registry of named map waypoints from two sources: a
 *        Lanelet2 OSM map (read-only, derived) and a dynamic waypoints.yaml
 *        (user-saved, persisted).
 *
 * Lookups prefer lanelet locations over dynamic ones on a name clash. All
 * accessors lock an internal mutex so BT action threads and ROS service handlers
 * can read/write concurrently.
 */
class LocationRegistry {
public:
  struct Location {
    std::string name;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    std::string lanelet_name;
    std::string source;  // "lanelet2" or "dynamic"
    std::string frame{"map"};
  };

  LocationRegistry() = default;
  ~LocationRegistry() = default;

  /**
   * @brief Load (replacing) the dynamic waypoint set from a waypoints.yaml file
   *        and remember the path for later persistence by save().
   * @param path Filesystem path to the YAML file; a missing file is a silent no-op.
   */
  void loadFromYaml(const std::string& path);

  /**
   * @brief Load named lanelets from a Lanelet2 OSM map, computing each lanelet's
   *        pose as the centre of its left/right boundary endpoints.
   * @param path Filesystem path to the .osm map.
   */
  void loadFromOsm(const std::string& path);

  /**
   * @brief Save (or overwrite) a dynamic waypoint and persist the dynamic set to
   *        the YAML path captured by loadFromYaml().
   * @param lanelet Optional associated lanelet name.
   * @return false only if a YAML path is set but the file could not be written;
   *         true otherwise (including when no path is configured).
   */
  bool save(const std::string& name, double x, double y, double theta, const std::string& lanelet = "");

  /**
   * @brief Look up a waypoint by name, preferring lanelet over dynamic sources.
   * @return The Location, or std::nullopt if the name is unknown.
   */
  std::optional<Location> find(const std::string& name) const;

  /** @brief All known waypoint names (lanelet + dynamic, de-duplicated, sorted). */
  std::vector<std::string> allNames() const;

private:
  std::map<std::string, Location> lanelet_locations_;
  std::map<std::string, Location> dynamic_locations_;
  std::string yaml_path_;
  mutable std::mutex mutex_;
};

}  // namespace rover_bt
