#include "nmpc_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

inline bool isOccupied(const int8_t value) { return value >= 99; }

inline std::size_t flattenIndex(const std::size_t row, const std::size_t col,
                                const std::size_t width) {
  return row * width + col;
}
} // namespace

NMPCController::NMPCController(const ControllerConfig &config)
    : config_(config) {}

bool NMPCController::initialize() {
  if (config_.N <= 1) {
    throw std::invalid_argument("NMPCController: config.N must be > 1");
  }

  if (config_.h <= 0.0) {
    throw std::invalid_argument("NMPCController: config.h must be > 0");
  }

  if (config_.L <= 0.0) {
    throw std::invalid_argument("NMPCController: config.L must be > 0");
  }

  if (config_.v_max <= 0.0) {
    throw std::invalid_argument("NMPCController: config.v_max must be > 0");
  }

  if (config_.a_max <= 0.0) {
    throw std::invalid_argument("NMPCController: config.a_max must be > 0");
  }

  if (config_.voxel_size <= 0.0) {
    throw std::invalid_argument(
        "NMPCController: config.voxel_size must be > 0");
  }

  if (config_.d_safe <= 0.0) {
    throw std::invalid_argument("NMPCController: config.d_safe must be > 0");
  }

  if (config_.max_range <= 0.0) {
    throw std::invalid_argument("NMPCController: config.max_range must be > 0");
  }

  setupBaseProblem();
  initialized_ = true;
  return true;
}

double NMPCController::normalizeAngle(double angle) {
  double normalized = std::fmod(angle, kTwoPi);
  if (normalized < 0.0) {
    normalized += kTwoPi;
  }
  return normalized;
}

void NMPCController::setupBaseProblem() {
  using casadi::MX;
  using casadi::Slice;
  using casadi::vertcat;

  const int nx = config_.N * n_;

  X_ = MX::sym("X", nx);

  const MX x = X_(Slice(0, nx, n_));
  const MX y = X_(Slice(1, nx, n_));
  const MX theta = X_(Slice(2, nx, n_));
  const MX vr = X_(Slice(3, nx, n_));
  const MX vl = X_(Slice(4, nx, n_));
  const MX ar = X_(Slice(5, nx, n_));
  const MX al = X_(Slice(6, nx, n_));

  lbx_.clear();
  ubx_.clear();
  lbx_.reserve(static_cast<std::size_t>(nx));
  ubx_.reserve(static_cast<std::size_t>(nx));

  for (int i = 0; i < config_.N; ++i) {
    lbx_.push_back(-std::numeric_limits<double>::infinity()); // x
    lbx_.push_back(-std::numeric_limits<double>::infinity()); // y
    lbx_.push_back(-std::numeric_limits<double>::infinity()); // theta
    lbx_.push_back(-config_.v_max);                           // vr
    lbx_.push_back(-config_.v_max);                           // vl
    lbx_.push_back(-config_.a_max);                           // ar
    lbx_.push_back(-config_.a_max);                           // al

    ubx_.push_back(std::numeric_limits<double>::infinity()); // x
    ubx_.push_back(std::numeric_limits<double>::infinity()); // y
    ubx_.push_back(std::numeric_limits<double>::infinity()); // theta
    ubx_.push_back(config_.v_max);                           // vr
    ubx_.push_back(config_.v_max);                           // vl
    ubx_.push_back(config_.a_max);                           // ar
    ubx_.push_back(config_.a_max);                           // al
  }

  const MX x_k = x(Slice(0, config_.N - 1));
  const MX x_k1 = x(Slice(1, config_.N));
  const MX y_k = y(Slice(0, config_.N - 1));
  const MX y_k1 = y(Slice(1, config_.N));
  const MX th_k = theta(Slice(0, config_.N - 1));
  const MX th_k1 = theta(Slice(1, config_.N));
  const MX vr_k = vr(Slice(0, config_.N - 1));
  const MX vr_k1 = vr(Slice(1, config_.N));
  const MX vl_k = vl(Slice(0, config_.N - 1));
  const MX vl_k1 = vl(Slice(1, config_.N));
  const MX ar_k = ar(Slice(0, config_.N - 1));
  const MX ar_k1 = ar(Slice(1, config_.N));
  const MX al_k = al(Slice(0, config_.N - 1));
  const MX al_k1 = al(Slice(1, config_.N));

  const MX gx = x_k1 - x_k -
                0.5 * config_.h *
                    ((((vr_k1 + vl_k1) / 2.0) * casadi::cos(th_k1)) +
                     (((vr_k + vl_k) / 2.0) * casadi::cos(th_k)));

  const MX gy = y_k1 - y_k -
                0.5 * config_.h *
                    ((((vr_k1 + vl_k1) / 2.0) * casadi::sin(th_k1)) +
                     (((vr_k + vl_k) / 2.0) * casadi::sin(th_k)));

  const MX gtheta =
      th_k1 - th_k -
      0.5 * config_.h *
          (((vr_k1 - vl_k1) / config_.L) + ((vr_k - vl_k) / config_.L));

  const MX gv_r = vr_k1 - vr_k - 0.5 * config_.h * (ar_k1 + ar_k);
  const MX gv_l = vl_k1 - vl_k - 0.5 * config_.h * (al_k1 + al_k);

  g_base_ = vertcat({gx, gy, gtheta, gv_r, gv_l});
  g_current_ = g_base_;

  opt_states_cache_.clear();

  DBG("Base NMPC problem created");
}

std::vector<Point2D>
NMPCController::extractObstacleVoxels(const OccupancyGridData &occupancy,
                                      const RobotState &x0) const {
  if (!occupancy.valid()) {
    return {};
  }

  const double heading = normalizeAngle(x0.theta);
  const double data_min = 0.2;
  const double data_max = 1.0 - data_min;
  const double mr_cells = config_.max_range / occupancy.resolution;

  int a_min = 0;
  int a_max = static_cast<int>(occupancy.width);
  int b_min = 0;
  int b_max = static_cast<int>(occupancy.height);

  const double cx = static_cast<double>(occupancy.width) / 2.0;
  const double cy = static_cast<double>(occupancy.height) / 2.0;

  if (((15.0 * kPi / 8.0) <= heading && heading < 2.0 * kPi) ||
      (0.0 <= heading && heading < (kPi / 8.0))) {
    a_min = static_cast<int>(std::round(cx - data_min * mr_cells));
    a_max = static_cast<int>(std::round(cx + data_max * mr_cells + 1.0));
    b_min = static_cast<int>(std::round(cy - mr_cells / 2.0));
    b_max = static_cast<int>(std::round(cy + mr_cells / 2.0 + 1.0));
  } else if ((kPi / 8.0) <= heading && heading < (3.0 * kPi / 8.0)) {
    a_min = static_cast<int>(std::round(cx - data_min * mr_cells));
    a_max = static_cast<int>(std::round(cx + data_max * mr_cells + 1.0));
    b_min = static_cast<int>(std::round(cy - data_min * mr_cells));
    b_max = static_cast<int>(std::round(cy + data_max * mr_cells + 1.0));
  } else if ((3.0 * kPi / 8.0) <= heading && heading < (5.0 * kPi / 8.0)) {
    a_min = static_cast<int>(std::round(cx - mr_cells / 2.0));
    a_max = static_cast<int>(std::round(cx + mr_cells / 2.0 + 1.0));
    b_min = static_cast<int>(std::round(cy - data_min * mr_cells));
    b_max = static_cast<int>(std::round(cy + data_max * mr_cells + 1.0));
  } else if ((5.0 * kPi / 8.0) <= heading && heading < (7.0 * kPi / 8.0)) {
    a_min = static_cast<int>(std::round(cx - data_max * mr_cells));
    a_max = static_cast<int>(std::round(cx + data_min * mr_cells + 1.0));
    b_min = static_cast<int>(std::round(cy - data_min * mr_cells));
    b_max = static_cast<int>(std::round(cy + data_max * mr_cells + 1.0));
  } else if ((7.0 * kPi / 8.0) <= heading && heading < (9.0 * kPi / 8.0)) {
    a_min = static_cast<int>(std::round(cx - data_max * mr_cells));
    a_max = static_cast<int>(std::round(cx + data_min * mr_cells + 1.0));
    b_min = static_cast<int>(std::round(cy - mr_cells / 2.0));
    b_max = static_cast<int>(std::round(cy + mr_cells / 2.0 + 1.0));
  } else if ((9.0 * kPi / 8.0) <= heading && heading < (11.0 * kPi / 8.0)) {
    a_min = static_cast<int>(std::round(cx - data_max * mr_cells));
    a_max = static_cast<int>(std::round(cx + data_min * mr_cells + 1.0));
    b_min = static_cast<int>(std::round(cy - data_max * mr_cells));
    b_max = static_cast<int>(std::round(cy + data_min * mr_cells + 1.0));
  } else if ((11.0 * kPi / 8.0) <= heading && heading < (13.0 * kPi / 8.0)) {
    a_min = static_cast<int>(std::round(cx - mr_cells / 2.0));
    a_max = static_cast<int>(std::round(cx + mr_cells / 2.0 + 1.0));
    b_min = static_cast<int>(std::round(cy - data_max * mr_cells));
    b_max = static_cast<int>(std::round(cy + data_min * mr_cells + 1.0));
  } else if ((13.0 * kPi / 8.0) <= heading && heading < (15.0 * kPi / 8.0)) {
    a_min = static_cast<int>(std::round(cx - data_min * mr_cells));
    a_max = static_cast<int>(std::round(cx + data_max * mr_cells + 1.0));
    b_min = static_cast<int>(std::round(cy - data_max * mr_cells));
    b_max = static_cast<int>(std::round(cy + data_min * mr_cells + 1.0));
  } else {
    a_min = static_cast<int>(std::round(cx - mr_cells / 2.0));
    a_max = static_cast<int>(std::round(cx + mr_cells / 2.0 + 1.0));
    b_min = static_cast<int>(std::round(cy - mr_cells / 2.0));
    b_max = static_cast<int>(std::round(cy + mr_cells / 2.0 + 1.0));
  }

  a_min = std::max(0, a_min);
  b_min = std::max(0, b_min);
  a_max = std::min(static_cast<int>(occupancy.width), a_max);
  b_max = std::min(static_cast<int>(occupancy.height), b_max);

  std::vector<Point2D> obstacle_points;
  obstacle_points.reserve(static_cast<std::size_t>(std::max(0, a_max - a_min) *
                                                   std::max(0, b_max - b_min)));

  for (int row = b_min; row < b_max; ++row) {
    for (int col = a_min; col < a_max; ++col) {
      const std::size_t idx =
          flattenIndex(static_cast<std::size_t>(row),
                       static_cast<std::size_t>(col), occupancy.width);

      if (!isOccupied(occupancy.data[idx])) {
        continue;
      }

      obstacle_points.push_back(Point2D{
          occupancy.origin_x + static_cast<double>(col) * occupancy.resolution,
          occupancy.origin_y +
              static_cast<double>(row) * occupancy.resolution});
    }
  }

  if (obstacle_points.empty()) {
    return {};
  }

  double min_x = obstacle_points.front().x;
  double max_x = obstacle_points.front().x;
  double min_y = obstacle_points.front().y;
  double max_y = obstacle_points.front().y;

  for (const auto &p : obstacle_points) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }

  std::set<std::pair<int, int>> occupied_voxels;

  for (const auto &p : obstacle_points) {
    const int ix =
        static_cast<int>(std::round((p.x - min_x) / config_.voxel_size));
    const int iy =
        static_cast<int>(std::round((p.y - min_y) / config_.voxel_size));
    occupied_voxels.emplace(ix, iy);
  }

  std::vector<Point2D> voxel_midpoints;
  voxel_midpoints.reserve(occupied_voxels.size());

  for (const auto &voxel : occupied_voxels) {
    voxel_midpoints.push_back(Point2D{
        (static_cast<double>(voxel.first) + 0.5) * config_.voxel_size + min_x,
        (static_cast<double>(voxel.second) + 0.5) * config_.voxel_size +
            min_y});
  }

  return voxel_midpoints;
}

void NMPCController::rebuildConstraintVector(
    const RobotState &x0, const std::vector<Point2D> &voxel_obstacles) {
  using casadi::MX;
  using casadi::Slice;
  using casadi::vertcat;

  const MX x = X_(Slice(0, config_.N * n_, n_));
  const MX y = X_(Slice(1, config_.N * n_, n_));
  const MX theta = X_(Slice(2, config_.N * n_, n_));
  const MX vr = X_(Slice(3, config_.N * n_, n_));
  const MX vl = X_(Slice(4, config_.N * n_, n_));

  const MX init_constraints =
      vertcat({x(0) - x0.x, y(0) - x0.y, theta(0) - x0.theta, vr(0) - x0.vr,
               vl(0) - x0.vl});

  std::vector<MX> obstacle_constraints;
  obstacle_constraints.reserve(static_cast<std::size_t>(config_.N) *
                               voxel_obstacles.size());

  for (int i = 0; i < config_.N; ++i) {
    for (const auto &obs : voxel_obstacles) {
      const MX dx = x(i) - obs.x;
      const MX dy = y(i) - obs.y;
      obstacle_constraints.push_back(dx * dx + dy * dy -
                                     (config_.d_safe * config_.d_safe));
    }
  }

  if (obstacle_constraints.empty()) {
    g_current_ = vertcat({g_base_, init_constraints});
  } else {
    g_current_ =
        vertcat({g_base_, init_constraints, vertcat(obstacle_constraints)});
  }
}

casadi::MX
NMPCController::buildCostFunction(std::size_t step,
                                  const TrajectoryReference &reference) const {
  using casadi::MX;
  using casadi::Slice;

  const MX x = X_(Slice(0, config_.N * n_, n_));
  const MX y = X_(Slice(1, config_.N * n_, n_));
  const MX ar = X_(Slice(5, config_.N * n_, n_));
  const MX al = X_(Slice(6, config_.N * n_, n_));

  MX J = 0;

  for (int i = 0; i < config_.N; ++i) {
    const std::size_t k = step + static_cast<std::size_t>(i);

    const MX ref_follow_error =
        (x(i) - reference.x.at(k)) * (x(i) - reference.x.at(k)) +
        (y(i) - reference.y.at(k)) * (y(i) - reference.y.at(k));

    MX successive_error = 0;
    if (i != (config_.N - 1)) {
      successive_error = (ar(i + 1) - ar(i)) * (ar(i + 1) - ar(i)) +
                         (al(i + 1) - al(i)) * (al(i + 1) - al(i));
    }

    J += ref_follow_error + config_.lambda_1 * successive_error;
  }

  return J;
}

casadi::DM
NMPCController::buildInitialGuess(std::size_t step,
                                  const TrajectoryReference &reference) const {
  std::vector<double> guess;
  guess.reserve(static_cast<std::size_t>(config_.N * n_));

  if (step == 0 ||
      opt_states_cache_.size() != static_cast<std::size_t>(config_.N * n_)) {
    for (int i = 0; i < config_.N; ++i) {
      const std::size_t k = step + static_cast<std::size_t>(i);

      guess.push_back(reference.x.at(k));
      guess.push_back(reference.y.at(k));
      guess.push_back(reference.theta.at(k));
      guess.push_back(reference.vr.at(k));
      guess.push_back(reference.vl.at(k));
      guess.push_back(reference.ar.at(k));
      guess.push_back(reference.al.at(k));
    }
  } else {
    for (std::size_t i = static_cast<std::size_t>(n_);
         i < opt_states_cache_.size(); ++i) {
      guess.push_back(opt_states_cache_.at(i));
    }

    const std::size_t k = step + static_cast<std::size_t>(config_.N) - 1;
    guess.push_back(reference.x.at(k));
    guess.push_back(reference.y.at(k));
    guess.push_back(reference.theta.at(k));
    guess.push_back(reference.vr.at(k));
    guess.push_back(reference.vl.at(k));
    guess.push_back(reference.ar.at(k));
    guess.push_back(reference.al.at(k));
  }

  return casadi::DM(guess);
}

std::pair<casadi::DM, casadi::DM> NMPCController::buildConstraintBounds(
    std::size_t obstacle_constraint_count) const {
  const std::size_t base_count = static_cast<std::size_t>(5 * (config_.N - 1));
  const std::size_t init_count = 5;
  const std::size_t total_count =
      base_count + init_count + obstacle_constraint_count;

  std::vector<double> lbg(total_count, 0.0);
  std::vector<double> ubg(total_count, 0.0);

  for (std::size_t i = base_count + init_count; i < total_count; ++i) {
    ubg[i] = std::numeric_limits<double>::infinity();
  }

  return {casadi::DM(lbg), casadi::DM(ubg)};
}

SolveResult NMPCController::solve(std::size_t step, std::size_t step_tot,
                                  const TrajectoryReference &reference,
                                  const RobotState &x0,
                                  const OccupancyGridData &occupancy) {
  using Clock = std::chrono::steady_clock;

  SolveResult result{};

  if (!initialized_) {
    result.message = "NMPCController is not initialized";
    return result;
  }

  if (!reference.valid()) {
    result.message = "Invalid trajectory reference";
    return result;
  }

  if (step > step_tot) {
    result.message = "Step exceeds total trajectory steps";
    return result;
  }

  if (step >= reference.size()) {
    result.message = "Step is outside reference range";
    return result;
  }

  if (reference.size() < static_cast<std::size_t>(config_.N)) {
    result.message = "Reference shorter than prediction horizon";
    return result;
  }

  if (step + static_cast<std::size_t>(config_.N) > reference.size()) {
    result.message =
        "Reference does not contain enough samples for the prediction horizon";
    return result;
  }

  const auto data_t0 = Clock::now();
  result.voxel_obstacles = extractObstacleVoxels(occupancy, x0);
  const auto data_t1 = Clock::now();

  result.data_time = std::chrono::duration<double>(data_t1 - data_t0).count();

  rebuildConstraintVector(x0, result.voxel_obstacles);

  const casadi::MX J = buildCostFunction(step, reference);
  const casadi::DM init_guess = buildInitialGuess(step, reference);

  const auto bounds = buildConstraintBounds(
      static_cast<std::size_t>(config_.N) * result.voxel_obstacles.size());

  const auto solver_t0 = Clock::now();

  casadi::Dict opts;
  opts["ipopt.print_level"] = 0;
  opts["print_time"] = 0;
  opts["expand"] = true;

  casadi::MXDict prob{{"f", J}, {"x", X_}, {"g", g_current_}};

  // TODO: esto todavía construye el solver en cada llamada.
  // Funciona para validar lógica, pero luego conviene moverlo a initialize()
  // con una formulación paramétrica fija.
  casadi::Function solver = casadi::nlpsol("solver", "ipopt", prob, opts);

  std::map<std::string, casadi::DM> arg;
  arg["x0"] = init_guess;
  arg["lbx"] = casadi::DM(lbx_);
  arg["ubx"] = casadi::DM(ubx_);
  arg["lbg"] = bounds.first;
  arg["ubg"] = bounds.second;

  std::map<std::string, casadi::DM> sol;
  try {
    sol = solver(arg);
  } catch (const std::exception &e) {
    result.message = std::string("CasADi/IPOPT failed: ") + e.what();
    return result;
  }

  const auto solver_t1 = Clock::now();
  result.solver_time =
      std::chrono::duration<double>(solver_t1 - solver_t0).count();

  const std::vector<double> sol_x = sol.at("x").nonzeros();
  opt_states_cache_ = sol_x;

  result.vr_horizon.reserve(static_cast<std::size_t>(config_.N));
  result.vl_horizon.reserve(static_cast<std::size_t>(config_.N));

  for (int i = 0; i < config_.N; ++i) {
    const std::size_t base = static_cast<std::size_t>(i * n_);
    result.vr_horizon.push_back(sol_x.at(base + 3));
    result.vl_horizon.push_back(sol_x.at(base + 4));
  }

  result.success = true;
  result.message = "NMPC solved successfully";

  (void)step_tot;
  return result;
}