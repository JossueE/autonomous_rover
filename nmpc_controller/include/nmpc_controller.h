#pragma once

#include <casadi/casadi.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
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

/// Simple 2D point expressed in the controller reference frame.
struct Point2D {
  double x{0.0};
  double y{0.0};
};

/// Tunable parameters that define the NMPC prediction model and constraints.
struct ControllerConfig {
  double h{0.2}; ///< Sampling time [s] used by the prediction model.
  int N{20};     ///< Number of states in the prediction horizon.

  double L{0.633};     ///< Track width [m] for the differential-drive model.
  double v_max{0.8};   ///< Maximum wheel linear velocity [m/s].
  double a_max{0.5};   ///< Maximum wheel linear acceleration [m/s^2].

  double lambda_1{0.25};    ///< Weight for smooth acceleration changes.
  double lambda_theta{1.0}; ///< Weight for heading tracking error.
  double lambda_v{0.1};     ///< Weight for wheel-speed tracking error.

  double d_safe{0.8};      ///< Minimum allowed distance to obstacle voxels [m].
  double voxel_size{0.5};  ///< Size of each obstacle voxel used in the NLP [m].
  double max_range{3.5};   ///< Range around the robot used to scan obstacles [m].
  int max_obstacles{20};   ///< Fixed number of obstacle slots passed to the NLP.
};

/// Measured or estimated robot state used to initialize each NMPC solve.
struct RobotState {
  double x{0.0};      ///< Robot x position [m].
  double y{0.0};      ///< Robot y position [m].
  double theta{0.0};  ///< Robot heading [rad].
  double vr{0.0};     ///< Right-wheel linear velocity [m/s].
  double vl{0.0};     ///< Left-wheel linear velocity [m/s].
};

/// Reference trajectory sampled at the controller period.
struct TrajectoryReference {
  std::vector<double> x;      ///< Reference x positions [m].
  std::vector<double> y;      ///< Reference y positions [m].
  std::vector<double> theta;  ///< Reference headings [rad].
  std::vector<double> vr;     ///< Reference right-wheel speeds [m/s].
  std::vector<double> vl;     ///< Reference left-wheel speeds [m/s].
  std::vector<double> ar;     ///< Reference right-wheel accelerations [m/s^2].
  std::vector<double> al;     ///< Reference left-wheel accelerations [m/s^2].

  [[nodiscard]] std::size_t size() const { return x.size(); }
  [[nodiscard]] bool empty() const { return x.empty(); }

  [[nodiscard]] bool valid() const {
    const std::size_t n = x.size();
    return n > 0 && y.size() == n && theta.size() == n && vr.size() == n &&
           vl.size() == n && ar.size() == n && al.size() == n;
  }
};

/// Lightweight occupancy-grid container used by the controller core.
struct OccupancyGridData {
  double origin_x{0.0};       ///< Map origin x coordinate [m].
  double origin_y{0.0};       ///< Map origin y coordinate [m].
  double resolution{0.05};    ///< Cell size [m/cell].
  std::size_t width{0};       ///< Grid width [cells].
  std::size_t height{0};      ///< Grid height [cells].
  std::vector<int8_t> data;   ///< Row-major occupancy values.

  [[nodiscard]] bool valid() const {
    return width > 0 && height > 0 && resolution > 0.0 &&
           data.size() == width * height;
  }
};

/// Result returned by a single NMPC optimization step.
struct SolveResult {
  bool success{false};
  std::string message;

  std::vector<double> vr_horizon;        ///< Predicted right-wheel speeds.
  std::vector<double> vl_horizon;        ///< Predicted left-wheel speeds.
  std::vector<Point2D> voxel_obstacles;  ///< Obstacle voxels used in the solve.

  double solver_time{0.0}; ///< Time spent inside the NLP solver [s].
  double data_time{0.0};   ///< Time spent preparing obstacle data [s].
};

/// Core NMPC solver for a differential-drive robot.
class NMPCController {
public:
  explicit NMPCController(const ControllerConfig &config = {});
  virtual ~NMPCController() = default;

  /// Build and compile the CasADi/IPOPT problem from the current config.
  bool initialize();

  /**
   * Solve one NMPC step for the current robot state.
   *
   * @param step Current reference index used as the first prediction sample.
   * @param step_tot Total number of samples available in the reference buffer.
   * @param reference Full reference trajectory used to populate the horizon.
   * @param x0 Current robot state used as the initial-condition constraint.
   * @param occupancy Current occupancy grid used for obstacle extraction.
   * @return SolveResult containing the predicted wheel-speed horizon and status.
   */
  [[nodiscard]] SolveResult solve(std::size_t step, std::size_t step_tot,
                                  const TrajectoryReference &reference,
                                  const RobotState &x0,
                                  const OccupancyGridData &occupancy);

  /// Read-only access to the active controller configuration.
  [[nodiscard]] const ControllerConfig &config() const { return config_; }
  /// Number of decision variables stored per horizon sample.
  [[nodiscard]] int stateDimension() const { return n_; }
  /// Prediction horizon length in samples.
  [[nodiscard]] int horizon() const { return config_.N; }
  /// True once initialize() has built the solver.
  [[nodiscard]] bool isInitialized() const { return initialized_; }

protected:
  /// Create the parameterized nonlinear program and solver instance.
  void setupBaseProblem();

  /**
   * Extract occupied cells around the robot and compress them into obstacle
   * voxels for the NLP.
   *
   * @param occupancy Current occupancy grid.
   * @param x0 Current robot state used to center the obstacle search.
   * @return List of obstacle voxel centroids in world coordinates.
   */
  [[nodiscard]] std::vector<Point2D>
  extractObstacleVoxels(const OccupancyGridData &occupancy,
                        const RobotState &x0) const;

  /**
   * Build the solver initial guess, reusing the previous solution whenever
   * possible to improve convergence.
   *
   * @param step First reference sample used by the current solve.
   * @param reference Full reference trajectory.
   * @return Dense CasADi vector with one state block per horizon sample.
   */
  [[nodiscard]] casadi::DM
  buildInitialGuess(std::size_t step,
                    const TrajectoryReference &reference) const;

  /// Wrap an angle to the [0, 2*pi) interval.
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
