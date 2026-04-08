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
// Sentinel position used for unused obstacle slots (far enough that the
// constraint dx^2+dy^2 - d_safe^2 >= 0 is always trivially satisfied).
constexpr double kObstacleSentinel = 1.0e6;

inline bool isOccupied(const int8_t value) { return value >= 99; }

inline std::size_t flattenIndex(const std::size_t row, const std::size_t col,
                                const std::size_t width) {
  return row * width + col;
}
} // namespace

// ─────────────────────────────────────────────────────────────────────────────
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
    throw std::invalid_argument(
        "NMPCController: config.max_range must be > 0");
  }
  if (config_.max_obstacles < 0) {
    throw std::invalid_argument(
        "NMPCController: config.max_obstacles must be >= 0");
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

// ─────────────────────────────────────────────────────────────────────────────
// setupBaseProblem
//
// Builds the full parameterized NLP once and compiles the solver.
//
// Parameter vector layout (size = p_size_):
//   p[0..4]              – initial state (x0, y0, θ0, vr0, vl0)
//   p[5 .. 5+5N-1]       – reference per step (x, y, θ, vr, vl) × N
//   p[5+5N .. p_size_-1] – obstacle positions (ox, oy) × max_obstacles
//
// Inactive obstacle slots carry the sentinel value (1e6, 1e6) so the
// avoidance constraint d^2 - d_safe^2 >= 0 is trivially satisfied.
// ─────────────────────────────────────────────────────────────────────────────
void NMPCController::setupBaseProblem() {
  using casadi::MX;
  using casadi::Slice;

  const int nx = config_.N * n_;

  p_size_ = 5 + 5 * config_.N + 2 * config_.max_obstacles;

  X_ = MX::sym("X", nx);
  p_ = MX::sym("p", p_size_);

  // ── State component views ──────────────────────────────────────────────────
  const MX x     = X_(Slice(0, nx, n_));
  const MX y     = X_(Slice(1, nx, n_));
  const MX theta = X_(Slice(2, nx, n_));
  const MX vr    = X_(Slice(3, nx, n_));
  const MX vl    = X_(Slice(4, nx, n_));
  const MX ar    = X_(Slice(5, nx, n_));
  const MX al    = X_(Slice(6, nx, n_));

  // ── Bounds ─────────────────────────────────────────────────────────────────
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

  // ── Dynamics constraints (trapezoidal / Heun integration) ─────────────────
  const MX x_k   = x(Slice(0, config_.N - 1));
  const MX x_k1  = x(Slice(1, config_.N));
  const MX y_k   = y(Slice(0, config_.N - 1));
  const MX y_k1  = y(Slice(1, config_.N));
  const MX th_k  = theta(Slice(0, config_.N - 1));
  const MX th_k1 = theta(Slice(1, config_.N));
  const MX vr_k  = vr(Slice(0, config_.N - 1));
  const MX vr_k1 = vr(Slice(1, config_.N));
  const MX vl_k  = vl(Slice(0, config_.N - 1));
  const MX vl_k1 = vl(Slice(1, config_.N));
  const MX ar_k  = ar(Slice(0, config_.N - 1));
  const MX ar_k1 = ar(Slice(1, config_.N));
  const MX al_k  = al(Slice(0, config_.N - 1));
  const MX al_k1 = al(Slice(1, config_.N));

  const MX gx =
      x_k1 - x_k -
      0.5 * config_.h *
          (((vr_k1 + vl_k1) / 2.0) * cos(th_k1) +
           ((vr_k + vl_k) / 2.0) * cos(th_k));

  const MX gy =
      y_k1 - y_k -
      0.5 * config_.h *
          (((vr_k1 + vl_k1) / 2.0) * sin(th_k1) +
           ((vr_k + vl_k) / 2.0) * sin(th_k));

  const MX gtheta =
      th_k1 - th_k -
      0.5 * config_.h *
          (((vr_k1 - vl_k1) / config_.L) + ((vr_k - vl_k) / config_.L));

  const MX gv_r = vr_k1 - vr_k - 0.5 * config_.h * (ar_k1 + ar_k);
  const MX gv_l = vl_k1 - vl_k - 0.5 * config_.h * (al_k1 + al_k);

  const MX g_dyn = MX::vertcat({gx, gy, gtheta, gv_r, gv_l});

  // ── Initial condition constraints (driven by parameter p[0..4]) ───────────
  const MX g_init = MX::vertcat({x(0) - p_(0), y(0) - p_(1),
                                  theta(0) - p_(2), vr(0) - p_(3),
                                  vl(0) - p_(4)});

  // ── Obstacle avoidance constraints (fixed count = max_obstacles) ──────────
  const int obs_offset = 5 + 5 * config_.N;
  std::vector<MX> obs_constr;
  obs_constr.reserve(
      static_cast<std::size_t>(config_.N * config_.max_obstacles));

  for (int j = 0; j < config_.max_obstacles; ++j) {
    const MX ox = p_(obs_offset + 2 * j);
    const MX oy = p_(obs_offset + 2 * j + 1);
    for (int i = 0; i < config_.N; ++i) {
      const MX dx = x(i) - ox;
      const MX dy = y(i) - oy;
      obs_constr.push_back(dx * dx + dy * dy -
                           config_.d_safe * config_.d_safe);
    }
  }

  MX g;
  if (obs_constr.empty()) {
    g = MX::vertcat({g_dyn, g_init});
  } else {
    g = MX::vertcat({g_dyn, g_init, MX::vertcat(obs_constr)});
  }

  // ── Cost function ──────────────────────────────────────────────────────────
  // p[5 + 5*i .. 5 + 5*i + 4] = (x_ref, y_ref, theta_ref, vr_ref, vl_ref)
  MX J = 0;
  for (int i = 0; i < config_.N; ++i) {
    const int    rb        = 5 + 5 * i;
    const MX     x_ref     = p_(rb + 0);
    const MX     y_ref     = p_(rb + 1);
    const MX     theta_ref = p_(rb + 2);
    const MX     vr_ref    = p_(rb + 3);
    const MX     vl_ref    = p_(rb + 4);

    const MX pos_err =
        (x(i) - x_ref) * (x(i) - x_ref) +
        (y(i) - y_ref) * (y(i) - y_ref);

    const MX heading_delta = theta(i) - theta_ref;
    const MX wrapped_heading_delta =
        atan2(sin(heading_delta), cos(heading_delta));
    const MX heading_err =
        config_.lambda_theta * wrapped_heading_delta * wrapped_heading_delta;

    const MX vel_err =
        config_.lambda_v *
        ((vr(i) - vr_ref) * (vr(i) - vr_ref) +
         (vl(i) - vl_ref) * (vl(i) - vl_ref));

    MX smooth_err = 0;
    if (i < config_.N - 1) {
      smooth_err =
          config_.lambda_1 *
          ((ar(i + 1) - ar(i)) * (ar(i + 1) - ar(i)) +
           (al(i + 1) - al(i)) * (al(i + 1) - al(i)));
    }

    J += pos_err + heading_err + vel_err + smooth_err;
  }

  // ── Compile solver once ────────────────────────────────────────────────────
  casadi::Dict opts;
  opts["ipopt.print_level"] = 0;
  opts["print_time"]        = 0;
  opts["expand"]            = true;

  const casadi::MXDict prob{{"f", J}, {"x", X_}, {"g", g}, {"p", p_}};
  solver_ = casadi::nlpsol("solver", "ipopt", prob, opts);

  opt_states_cache_.clear();
  DBG("NMPC problem built and solver compiled (parameterized)");
}

// ─────────────────────────────────────────────────────────────────────────────
// extractObstacleVoxels  (unchanged from original)
// ─────────────────────────────────────────────────────────────────────────────
std::vector<Point2D>
NMPCController::extractObstacleVoxels(const OccupancyGridData &occupancy,
                                      const RobotState &x0) const {
  if (!occupancy.valid()) {
    return {};
  }

  const double heading  = normalizeAngle(x0.theta);
  const double data_min = 0.2;
  const double data_max = 1.0 - data_min;
  const double mr_cells = config_.max_range / occupancy.resolution;

  int a_min = 0;
  int a_max = static_cast<int>(occupancy.width);
  int b_min = 0;
  int b_max = static_cast<int>(occupancy.height);

  const double cx = std::clamp((x0.x - occupancy.origin_x) /
                                   occupancy.resolution,
                               0.0,
                               static_cast<double>(occupancy.width - 1));
  const double cy = std::clamp((x0.y - occupancy.origin_y) /
                                   occupancy.resolution,
                               0.0,
                               static_cast<double>(occupancy.height - 1));

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

// ─────────────────────────────────────────────────────────────────────────────
// buildInitialGuess  (unchanged from original)
// ─────────────────────────────────────────────────────────────────────────────
casadi::DM
NMPCController::buildInitialGuess(std::size_t step,
                                  const TrajectoryReference &reference) const {
  std::vector<double> guess;
  guess.reserve(static_cast<std::size_t>(config_.N * n_));
  const std::size_t last = reference.size() - 1;

  if (step == 0 ||
      opt_states_cache_.size() != static_cast<std::size_t>(config_.N * n_)) {
    for (int i = 0; i < config_.N; ++i) {
      const std::size_t k =
          std::min(step + static_cast<std::size_t>(i), last);
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
    const std::size_t k =
        std::min(step + static_cast<std::size_t>(config_.N) - 1, last);
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

// ─────────────────────────────────────────────────────────────────────────────
// solve
// ─────────────────────────────────────────────────────────────────────────────
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

  // BUG FIX #3: was `step > step_tot` (off-by-one — step == step_tot is OOB)
  if (step >= step_tot) {
    result.message = "Step exceeds total trajectory steps";
    return result;
  }

  if (step >= reference.size()) {
    result.message = "Step is outside reference range";
    return result;
  }

  // ── Extract obstacle voxels ────────────────────────────────────────────────
  const auto data_t0 = Clock::now();
  result.voxel_obstacles = extractObstacleVoxels(occupancy, x0);
  const auto data_t1 = Clock::now();
  result.data_time = std::chrono::duration<double>(data_t1 - data_t0).count();

  // Sort by proximity, then cap at max_obstacles (closest take priority)
  if (result.voxel_obstacles.size() >
      static_cast<std::size_t>(config_.max_obstacles)) {
    std::sort(result.voxel_obstacles.begin(), result.voxel_obstacles.end(),
              [&x0](const Point2D &a, const Point2D &b) {
                const double da = (a.x - x0.x) * (a.x - x0.x) +
                                  (a.y - x0.y) * (a.y - x0.y);
                const double db = (b.x - x0.x) * (b.x - x0.x) +
                                  (b.y - x0.y) * (b.y - x0.y);
                return da < db;
              });
    result.voxel_obstacles.resize(
        static_cast<std::size_t>(config_.max_obstacles));
  }

  // ── Build parameter vector ────────────────────────────────────────────────
  std::vector<double> p_val;
  p_val.reserve(static_cast<std::size_t>(p_size_));

  // p[0..4] = initial robot state
  p_val.push_back(x0.x);
  p_val.push_back(x0.y);
  p_val.push_back(x0.theta);
  p_val.push_back(x0.vr);
  p_val.push_back(x0.vl);

  const std::size_t last = reference.size() - 1;

  // p[5 .. 5+5N-1] = reference trajectory
  for (int i = 0; i < config_.N; ++i) {
    const std::size_t k =
        std::min(step + static_cast<std::size_t>(i), last);
    p_val.push_back(reference.x.at(k));
    p_val.push_back(reference.y.at(k));
    p_val.push_back(reference.theta.at(k));
    p_val.push_back(reference.vr.at(k));
    p_val.push_back(reference.vl.at(k));
  }

  // p[5+5N ..] = obstacle positions (unused slots → sentinel)
  const std::size_t n_obs = result.voxel_obstacles.size();
  for (int j = 0; j < config_.max_obstacles; ++j) {
    if (static_cast<std::size_t>(j) < n_obs) {
      p_val.push_back(result.voxel_obstacles[j].x);
      p_val.push_back(result.voxel_obstacles[j].y);
    } else {
      p_val.push_back(kObstacleSentinel);
      p_val.push_back(kObstacleSentinel);
    }
  }

  // ── Constraint bounds (fixed structure) ───────────────────────────────────
  const std::size_t dyn_count  = static_cast<std::size_t>(5 * (config_.N - 1));
  const std::size_t init_count = 5;
  const std::size_t obs_count  =
      static_cast<std::size_t>(config_.N * config_.max_obstacles);
  const std::size_t total_g = dyn_count + init_count + obs_count;

  std::vector<double> lbg(total_g, 0.0);
  std::vector<double> ubg(total_g, 0.0);
  // Obstacle constraints are inequalities: g >= 0
  for (std::size_t i = dyn_count + init_count; i < total_g; ++i) {
    ubg[i] = std::numeric_limits<double>::infinity();
  }

  // ── Initial guess (warm-start from previous solution) ─────────────────────
  const casadi::DM init_guess = buildInitialGuess(step, reference);

  // ── Call the pre-compiled solver ──────────────────────────────────────────
  const auto solver_t0 = Clock::now();

  std::map<std::string, casadi::DM> arg;
  arg["x0"]  = init_guess;
  arg["lbx"] = casadi::DM(lbx_);
  arg["ubx"] = casadi::DM(ubx_);
  arg["lbg"] = casadi::DM(lbg);
  arg["ubg"] = casadi::DM(ubg);
  arg["p"]   = casadi::DM(p_val);

  std::map<std::string, casadi::DM> sol;
  try {
    sol = solver_(arg);
  } catch (const std::exception &e) {
    result.message = std::string("CasADi/IPOPT failed: ") + e.what();
    return result;
  }

  const casadi::Dict stats = solver_.stats();
  const bool solver_success = casadi::get_from_dict(stats, "success", false);
  const std::string return_status =
      casadi::get_from_dict(stats, "return_status", std::string{});

  const auto solver_t1 = Clock::now();
  result.solver_time =
      std::chrono::duration<double>(solver_t1 - solver_t0).count();

  if (!solver_success) {
    result.message = return_status.empty()
                         ? "IPOPT reported an unsuccessful solve"
                         : "IPOPT reported an unsuccessful solve: " +
                               return_status;
    return result;
  }

  const std::vector<double> sol_x = sol.at("x").nonzeros();
  if (sol_x.size() != static_cast<std::size_t>(config_.N * n_)) {
    result.message = "Solver returned an unexpected decision vector size";
    return result;
  }
  opt_states_cache_ = sol_x;

  result.vr_horizon.reserve(static_cast<std::size_t>(config_.N));
  result.vl_horizon.reserve(static_cast<std::size_t>(config_.N));

  for (int i = 0; i < config_.N; ++i) {
    const std::size_t base = static_cast<std::size_t>(i * n_);
    result.vr_horizon.push_back(sol_x.at(base + 3));
    result.vl_horizon.push_back(sol_x.at(base + 4));
  }

  result.success = true;
  result.message = return_status.empty() ? "NMPC solved successfully"
                                         : return_status;

  (void)step_tot;
  return result;
}
