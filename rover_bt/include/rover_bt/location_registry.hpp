#pragma once

#include <string>
#include <vector>
#include <map>
#include <optional>
#include <mutex>

namespace rover_bt {

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

  void loadFromYaml(const std::string& path);
  bool save(const std::string& name, double x, double y, double theta, const std::string& lanelet = "");
  std::optional<Location> find(const std::string& name) const;
  std::vector<std::string> allNames() const;

private:
  std::map<std::string, Location> lanelet_locations_;
  std::map<std::string, Location> dynamic_locations_;
  std::string yaml_path_;
  mutable std::mutex mutex_;
};

}  // namespace rover_bt
