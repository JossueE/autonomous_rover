#include "rover_bt/location_registry.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

namespace rover_bt {

static std::string trim(const std::string& str) {
  size_t first = str.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  size_t last = str.find_last_not_of(" \t\r\n");
  return str.substr(first, last - first + 1);
}

void LocationRegistry::loadFromYaml(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  yaml_path_ = path;
  std::ifstream fs(path);
  if (!fs.is_open()) {
    return;
  }

  dynamic_locations_.clear();
  std::string line;
  std::string current_name = "";
  Location current_loc;

  while (std::getline(fs, line)) {
    // Count leading spaces
    size_t indent = 0;
    while (indent < line.size() && line[indent] == ' ') {
      indent++;
    }

    std::string trimmed = trim(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }

    if (trimmed == "waypoints:") {
      continue;
    }

    if (indent == 2 && trimmed.back() == ':') {
      // New waypoint name
      if (!current_name.empty()) {
        dynamic_locations_[current_name] = current_loc;
      }
      current_name = trimmed.substr(0, trimmed.size() - 1);
      current_loc = Location();
      current_loc.name = current_name;
      current_loc.source = "dynamic";
    } else if (indent == 4 && !current_name.empty()) {
      // Waypoint property
      size_t colon = trimmed.find(':');
      if (colon != std::string::npos) {
        std::string key = trim(trimmed.substr(0, colon));
        std::string val = trim(trimmed.substr(colon + 1));
        if (key == "x") {
          current_loc.x = std::stod(val);
        } else if (key == "y") {
          current_loc.y = std::stod(val);
        } else if (key == "theta") {
          current_loc.theta = std::stod(val);
        } else if (key == "lanelet") {
          // Remove potential quotes
          if (val.size() >= 2 && val.front() == '"' && val.back() == '"') {
            val = val.substr(1, val.size() - 2);
          }
          current_loc.lanelet_name = val;
        } else if (key == "frame") {
          if (val.size() >= 2 && val.front() == '"' && val.back() == '"') {
            val = val.substr(1, val.size() - 2);
          }
          current_loc.frame = val;
        }
      }
    }
  }

  if (!current_name.empty()) {
    dynamic_locations_[current_name] = current_loc;
  }
}

bool LocationRegistry::save(const std::string& name, double x, double y, double theta, const std::string& lanelet) {
  std::lock_guard<std::mutex> lock(mutex_);
  Location loc;
  loc.name = name;
  loc.x = x;
  loc.y = y;
  loc.theta = theta;
  loc.lanelet_name = lanelet;
  loc.source = "dynamic";
  loc.frame = "map";

  dynamic_locations_[name] = loc;

  // Persist to waypoints.yaml
  if (yaml_path_.empty()) {
    return true;
  }

  std::ofstream out(yaml_path_);
  if (!out.is_open()) {
    return false;
  }

  out << "waypoints:\n";
  for (const auto& pair : dynamic_locations_) {
    out << "  " << pair.first << ":\n";
    out << "    x: " << pair.second.x << "\n";
    out << "    y: " << pair.second.y << "\n";
    out << "    theta: " << pair.second.theta << "\n";
    if (!pair.second.lanelet_name.empty()) {
      out << "    lanelet: \"" << pair.second.lanelet_name << "\"\n";
    }
    out << "    frame: \"" << pair.second.frame << "\"\n";
  }

  return true;
}

std::optional<LocationRegistry::Location> LocationRegistry::find(const std::string& name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = lanelet_locations_.find(name);
  if (it != lanelet_locations_.end()) {
    return it->second;
  }
  it = dynamic_locations_.find(name);
  if (it != dynamic_locations_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<std::string> LocationRegistry::allNames() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> names;
  for (const auto& pair : lanelet_locations_) {
    names.push_back(pair.first);
  }
  for (const auto& pair : dynamic_locations_) {
    if (lanelet_locations_.find(pair.first) == lanelet_locations_.end()) {
      names.push_back(pair.first);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

}  // namespace rover_bt
