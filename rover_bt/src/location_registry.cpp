#include "rover_bt/location_registry.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
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

void LocationRegistry::loadFromOsm(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::ifstream fs(path);
  if (!fs.is_open()) {
    std::cerr << "[LocationRegistry] Could not open OSM map: " << path << "\n";
    return;
  }

  // Extract an XML attribute value from a line: key="value"
  auto xmlAttr = [](const std::string& line, const std::string& key) -> std::string {
    auto pos = line.find(key + "=\"");
    if (pos == std::string::npos) return "";
    pos += key.size() + 2;
    auto end = line.find('"', pos);
    if (end == std::string::npos) return "";
    return line.substr(pos, end - pos);
  };

  // Pass 1: collect nodes  id → (local_x, local_y)
  struct NodeXY { double x = 0.0, y = 0.0; };
  std::map<std::string, NodeXY> node_map;

  // Pass 2: collect ways  id → ordered node id list
  std::map<std::string, std::vector<std::string>> way_map;

  // Pass 3: collect named lanelets  name → (left_way_id, right_way_id)
  struct LaneletDef { std::string name, left_way, right_way; };
  std::vector<LaneletDef> named_lanelets;

  // --- Single-pass state machine ---
  enum class St { None, Node, Way, Relation };
  St state = St::None;
  std::string cur_id;
  NodeXY cur_node;
  std::vector<std::string> cur_way_nodes;
  LaneletDef cur_rel;
  bool cur_is_lanelet = false;

  std::string line;
  while (std::getline(fs, line)) {
    const std::string t = trim(line);

    if (t.rfind("<node ", 0) == 0) {
      state = St::Node;
      cur_id = xmlAttr(t, "id");
      cur_node = {};
    } else if (t == "</node>") {
      if (!cur_id.empty()) node_map[cur_id] = cur_node;
      state = St::None;
    } else if (t.rfind("<way ", 0) == 0) {
      state = St::Way;
      cur_id = xmlAttr(t, "id");
      cur_way_nodes.clear();
    } else if (t == "</way>") {
      if (!cur_id.empty()) way_map[cur_id] = cur_way_nodes;
      state = St::None;
    } else if (t.rfind("<relation ", 0) == 0) {
      state = St::Relation;
      cur_id = xmlAttr(t, "id");
      cur_rel = {};
      cur_is_lanelet = false;
    } else if (t == "</relation>") {
      if (cur_is_lanelet && !cur_rel.name.empty() &&
          !cur_rel.left_way.empty() && !cur_rel.right_way.empty()) {
        named_lanelets.push_back(cur_rel);
      }
      state = St::None;
    } else if (state == St::Node && t.rfind("<tag ", 0) == 0) {
      const std::string k = xmlAttr(t, "k");
      const std::string v = xmlAttr(t, "v");
      if (k == "local_x") cur_node.x = std::stod(v);
      else if (k == "local_y") cur_node.y = std::stod(v);
    } else if (state == St::Way && t.rfind("<nd ", 0) == 0) {
      const std::string ref = xmlAttr(t, "ref");
      if (!ref.empty()) cur_way_nodes.push_back(ref);
    } else if (state == St::Relation && t.rfind("<member ", 0) == 0) {
      if (xmlAttr(t, "type") == "way") {
        const std::string role = xmlAttr(t, "role");
        const std::string ref  = xmlAttr(t, "ref");
        if (role == "left")  cur_rel.left_way  = ref;
        if (role == "right") cur_rel.right_way = ref;
      }
    } else if (state == St::Relation && t.rfind("<tag ", 0) == 0) {
      const std::string k = xmlAttr(t, "k");
      const std::string v = xmlAttr(t, "v");
      if (k == "type" && v == "lanelet") cur_is_lanelet = true;
      if (k == "name") cur_rel.name = v;
    }
  }

  // --- Compute x/y/theta for each named lanelet ---
  int loaded = 0;
  for (const auto& ll : named_lanelets) {
    auto lit = way_map.find(ll.left_way);
    auto rit = way_map.find(ll.right_way);
    if (lit == way_map.end() || rit == way_map.end()) continue;

    const auto& lnodes = lit->second;
    const auto& rnodes = rit->second;
    if (lnodes.empty() || rnodes.empty()) continue;

    // Centerline: average of left and right boundary start/end points
    auto getNode = [&](const std::string& id) -> NodeXY {
      auto it = node_map.find(id);
      return (it != node_map.end()) ? it->second : NodeXY{};
    };

    const NodeXY l0 = getNode(lnodes.front());
    const NodeXY lN = getNode(lnodes.back());
    const NodeXY r0 = getNode(rnodes.front());
    const NodeXY rN = getNode(rnodes.back());

    const double cx0 = (l0.x + r0.x) * 0.5;
    const double cy0 = (l0.y + r0.y) * 0.5;
    const double cxN = (lN.x + rN.x) * 0.5;
    const double cyN = (lN.y + rN.y) * 0.5;

    Location loc;
    loc.name        = ll.name;
    loc.x           = (cx0 + cxN) * 0.5;
    loc.y           = (cy0 + cyN) * 0.5;
    loc.theta       = std::atan2(cyN - cy0, cxN - cx0);
    loc.lanelet_name = ll.name;
    loc.source      = "lanelet2";
    loc.frame       = "map";

    lanelet_locations_[ll.name] = loc;
    ++loaded;
  }

  std::cout << "[LocationRegistry] Loaded " << loaded
            << " named lanelet(s) from " << path << "\n";
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
