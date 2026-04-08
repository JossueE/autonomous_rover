#pragma once

#include <casadi/casadi.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#ifdef NMPC_DEBUG
#define DBG(msg)                                                               \
  do {                                                                         \
    std::cerr << "[Debug] " << msg << '\n';                                    \
  } while (0)
#else
#define DBG(msg)                                                               \
  do {                                                                         \
  } while (0)
#endif

struct Point2D {
  double x{0.0};
  double y{0.0};
};

struct ControllerConfig {
  double h{0.2};
  int N{20};

  double L{0.633};
  double v_max{0.8};
  double a_max{0.5};

  double lambda_1{0.25};    // smoothness (successive acceleration change)
  double lambda_theta{1.0}; // heading tracking weight
  double lambda_v{0.1};     // wheel-velocity tracking weight

  double d_safe{0.8};
  double voxel_size{0.5};
  double max_range{3.5};
  int max_obstacles{20}; // NLP obstacle slots (extra slots use sentinel)
};

struct RobotState {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double vr{0.0};
  double vl{0.0};
};

struct TrajectoryReference {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> theta;
  std::vector<double> vr;
  std::vector<double> vl;
  std::vector<double> ar;
  std::vector<double> al;

  [[nodiscard]] std::size_t size() const { return x.size(); }
  [[nodiscard]] bool empty() const { return x.empty(); }

  [[nodiscard]] bool valid() const {
    const std::size_t n = x.size();
    return n > 0 && y.size() == n && theta.size() == n && vr.size() == n &&
           vl.size() == n && ar.size() == n && al.size() == n;
  }
};

struct OccupancyGridData {
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.05};
  std::size_t width{0};
  std::size_t height{0};
  std::vector<int8_t> data;

  [[nodiscard]] bool valid() const {
    return width > 0 && height > 0 && resolution > 0.0 &&
           data.size() == width * height;
  }
};

struct SolveResult {
  bool success{false};
  std::string message;

  std::vector<double> vr_horizon;
  std::vector<double> vl_horizon;
  std::vector<Point2D> voxel_obstacles;

  double solver_time{0.0};
  double data_time{0.0};
};

class NMPCController {
public:
  explicit NMPCController(const ControllerConfig &config = {});
  virtual ~NMPCController() = default;

  bool initialize();

  [[nodiscard]] SolveResult solve(std::size_t step, std::size_t step_tot,
                                  const TrajectoryReference &reference,
                                  const RobotState &x0,
                                  const OccupancyGridData &occupancy);

  [[nodiscard]] const ControllerConfig &config() const { return config_; }
  [[nodiscard]] int stateDimension() const { return n_; }
  [[nodiscard]] int horizon() const { return config_.N; }
  [[nodiscard]] bool isInitialized() const { return initialized_; }

protected:
  void setupBaseProblem();

  [[nodiscard]] std::vector<Point2D>
  extractObstacleVoxels(const OccupancyGridData &occupancy,
                        const RobotState &x0) const;

  [[nodiscard]] casadi::DM
  buildInitialGuess(std::size_t step,
                    const TrajectoryReference &reference) const;

  [[nodiscard]] static double normalizeAngle(double angle);

protected:
  ControllerConfig config_{};

  int n_{7};      // [x, y, theta, vr, vl, ar, al]
  int p_size_{0}; // total parameter count
  bool initialized_{false};

  casadi::MX X_;            // decision variable
  casadi::MX p_;            // parameter vector
  casadi::Function solver_; // compiled NLP solver (built once)

  std::vector<double> lbx_;
  std::vector<double> ubx_;
  std::vector<double> opt_states_cache_;
};