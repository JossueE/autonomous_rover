#!/usr/bin/env bash
set -e
set -u
set -o pipefail

REPO_ROOT=""
ROS_DISTRO_DETECTED=""
ROS_SETUP_FILE=""

info() { echo "[INFO] $*"; }
ok() { echo "[OK] $*"; }
warn() { echo "[WARN] $*"; }
error() { echo "[ERROR] $*" >&2; }

print_help() {
  cat <<'EOF'
Autonomous Rover installer

Usage:
  ./install.sh [--help]

This script assumes ROS2 is already installed. It detects the ROS2 distro,
uses rosdep for ROS dependencies, optionally installs Python requirements and
frontend npm dependencies, builds the workspace with colcon, and verifies
install/setup.bash.

Options:
  --help    Show this help message.

Notes:
  - The script does not install ROS2.
  - Heavy Python requirements and npm dependencies require confirmation.
  - Missing installer tools may be installed with apt only after confirmation.
EOF
}

ask_yes_no() {
  local prompt="$1"
  local answer=""
  read -r -p "$prompt [y/N] " answer || true
  case "$answer" in
    y|Y|yes|YES) return 0 ;;
    *) return 1 ;;
  esac
}

detect_repo_root() {
  REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  if [ ! -f "$REPO_ROOT/README.md" ]; then
    error "Could not detect repository root from script location."
    exit 1
  fi
  ok "Repository root: $REPO_ROOT"
}

check_os() {
  info "Checking operating system..."
  if [ -f /etc/os-release ]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    info "Detected OS: ${PRETTY_NAME:-unknown}"
    if [ "${ID:-}" != "ubuntu" ]; then
      warn "This repository is developed for Ubuntu/ROS2. Continuing anyway."
    fi
  else
    warn "Could not read /etc/os-release. Continuing with current system."
  fi
}

detect_ros_distro() {
  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    ROS_DISTRO_DETECTED="$ROS_DISTRO"
    ROS_SETUP_FILE="/opt/ros/${ROS_DISTRO_DETECTED}/setup.bash"
    return
  fi

  if [ -d /opt/ros ]; then
    local distro=""
    distro="$(find /opt/ros -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort | tail -n 1 || true)"
    if [ -n "$distro" ] && [ -f "/opt/ros/${distro}/setup.bash" ]; then
      ROS_DISTRO_DETECTED="$distro"
      ROS_SETUP_FILE="/opt/ros/${ROS_DISTRO_DETECTED}/setup.bash"
      return
    fi
  fi

  error "No ROS2 distro was detected."
  error "Install ROS2 first, then run: source /opt/ros/<distro>/setup.bash"
  exit 1
}

check_ros2() {
  info "Detecting ROS2..."
  detect_ros_distro
  if [ ! -f "$ROS_SETUP_FILE" ]; then
    error "Expected ROS2 setup file not found: $ROS_SETUP_FILE"
    exit 1
  fi

  # shellcheck disable=SC1090
  . "$ROS_SETUP_FILE"

  if ! command -v ros2 >/dev/null 2>&1; then
    error "ROS2 command not found after sourcing $ROS_SETUP_FILE."
    exit 1
  fi

  ok "ROS2 detected: ${ROS_DISTRO_DETECTED}"
}

install_system_dependencies() {
  info "Checking installer tools..."
  local missing_apt=()

  command -v python3 >/dev/null 2>&1 || missing_apt+=("python3")
  command -v pip3 >/dev/null 2>&1 || missing_apt+=("python3-pip")
  command -v rosdep >/dev/null 2>&1 || missing_apt+=("python3-rosdep")
  command -v colcon >/dev/null 2>&1 || missing_apt+=("python3-colcon-common-extensions")
  command -v tmux >/dev/null 2>&1 || missing_apt+=("tmux")

  if [ "${#missing_apt[@]}" -eq 0 ]; then
    ok "Required installer tools are available."
  else
    warn "Missing apt packages: ${missing_apt[*]}"
    if command -v apt-get >/dev/null 2>&1 && ask_yes_no "Install these packages with sudo apt-get?"; then
      info "Running: sudo apt-get update"
      sudo apt-get update
      info "Running: sudo apt-get install -y ${missing_apt[*]}"
      sudo apt-get install -y "${missing_apt[@]}"
    else
      error "Missing required tools. Install them manually and rerun this script."
      exit 1
    fi
  fi

  if command -v npm >/dev/null 2>&1; then
    ok "npm detected."
  else
    warn "npm was not found. Frontend dependencies will be skipped unless npm is installed."
  fi
}

install_ros_dependencies() {
  info "Installing ROS dependencies with rosdep..."

  if ! command -v rosdep >/dev/null 2>&1; then
    error "rosdep is not installed."
    exit 1
  fi

  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    warn "rosdep has not been initialized on this system."
    if ask_yes_no "Run sudo rosdep init?"; then
      sudo rosdep init
    else
      error "rosdep initialization is required."
      exit 1
    fi
  fi

  info "Updating rosdep database..."
  rosdep update

  # shellcheck disable=SC1090
  . "$ROS_SETUP_FILE"

  rosdep install --from-paths "$REPO_ROOT" --ignore-src -r -y
  ok "rosdep completed."
}

install_python_dependencies() {
  info "Checking Python requirements..."
  local requirements=()
  while IFS= read -r file; do
    requirements+=("$file")
  done < <(find "$REPO_ROOT" -path '*/node_modules' -prune -o -type f -name requirements.txt -print | sort)

  if [ "${#requirements[@]}" -eq 0 ]; then
    warn "No requirements.txt files found."
    return
  fi

  info "Found Python requirements:"
  printf '  %s\n' "${requirements[@]}"
  warn "Some requirements are large or hardware/perception specific."

  if ! ask_yes_no "Install all detected Python requirements with pip --user?"; then
    warn "Skipping Python requirements."
    return
  fi

  if ! command -v python3 >/dev/null 2>&1; then
    error "python3 is required for pip installation."
    exit 1
  fi

  for req in "${requirements[@]}"; do
    info "Installing Python requirements from $req"
    python3 -m pip install --user -r "$req"
  done
  ok "Python requirements processed."
}

install_frontend_dependencies() {
  local frontend_dir="$REPO_ROOT/front_end/ares-command-hub-main"
  local lockfile="$frontend_dir/package-lock.json"

  if [ ! -f "$lockfile" ]; then
    warn "No A.R.E.S. Command Hub package-lock.json found."
    return
  fi

  if ! command -v npm >/dev/null 2>&1; then
    warn "npm is not installed. Skipping frontend dependencies."
    return
  fi

  if ! ask_yes_no "Run npm install in $frontend_dir?"; then
    warn "Skipping npm install."
    return
  fi

  info "Installing frontend dependencies..."
  (cd "$frontend_dir" && npm install)
  ok "Frontend dependencies installed."
}

build_workspace() {
  info "Detecting ROS2 package build types..."
  local ament_python_count=0
  local ament_cmake_count=0

  ament_python_count="$(grep -R "<build_type>ament_python</build_type>" "$REPO_ROOT"/*/package.xml 2>/dev/null || true)"
  ament_python_count="$(printf '%s\n' "$ament_python_count" | sed '/^$/d' | wc -l | tr -d ' ')"
  ament_cmake_count="$(grep -R "<build_type>ament_cmake</build_type>" "$REPO_ROOT"/*/package.xml 2>/dev/null || true)"
  ament_cmake_count="$(printf '%s\n' "$ament_cmake_count" | sed '/^$/d' | wc -l | tr -d ' ')"

  info "ament_python packages: $ament_python_count"
  info "ament_cmake packages: $ament_cmake_count"

  if ! command -v colcon >/dev/null 2>&1; then
    error "colcon is not installed."
    exit 1
  fi

  # shellcheck disable=SC1090
  . "$ROS_SETUP_FILE"

  info "Building workspace with colcon build --symlink-install..."
  (cd "$REPO_ROOT" && colcon build --symlink-install)
  ok "Workspace build finished."
}

verify_installation() {
  info "Verifying installation..."

  if [ ! -f "$REPO_ROOT/install/setup.bash" ]; then
    error "install/setup.bash was not generated."
    exit 1
  fi

  # shellcheck disable=SC1091
  . "$REPO_ROOT/install/setup.bash"

  local packages=(
    rover_bringup
    rover_bt
    path_planning_dynamic
    rover_simulation
    teleop
  )

  for pkg in "${packages[@]}"; do
    if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
      ok "ROS2 package available: $pkg"
    else
      warn "Package not found after build: $pkg"
    fi
  done

  ok "Installation verified."
  echo
  echo "Next steps:"
  echo "  source install/setup.bash"
  echo "  ros2 launch rover_bringup bringup_all.launch.py"
  echo "  ros2 run rover_bringup bringup_all_tui.sh --restart"
  echo "  ros2 launch rover_simulation sim_bringup.launch.py"
}

main() {
  if [ "${1:-}" = "--help" ]; then
    print_help
    exit 0
  fi

  if [ "${1:-}" != "" ]; then
    error "Unknown option: $1"
    print_help
    exit 1
  fi

  detect_repo_root
  check_os
  check_ros2
  install_system_dependencies
  install_ros_dependencies
  install_python_dependencies
  install_frontend_dependencies
  build_workspace
  verify_installation
}

main "$@"
