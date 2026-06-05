#!/usr/bin/env bash
set -euo pipefail

SESSION="${ROVER_TUI_SESSION:-rover_bringup}"
LOG_DIR="${ROVER_TUI_LOG_DIR:-/tmp/rover_bringup_tui}"
LOG_FILE="${LOG_DIR}/bringup_all.log"
ROS_LOG_DIR_TUI="${LOG_DIR}/ros"
VOICE_REGEX="${ROVER_TUI_VOICE_REGEX:-voice_command_node|Wake_Word|Speech_To_Text|Text-to-Speech|\\[(wake_word|stt|tts|TTS|voice|Speak)\\]|TTSClient}"
RESTART=0
CREATED_SESSION=0

if [ "${1:-}" = "--restart" ]; then
  RESTART=1
  shift
fi

quote() {
  printf '%q' "$1"
}

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required. Install it with: sudo apt install tmux" >&2
  exit 1
fi

mkdir -p "$LOG_DIR" "$ROS_LOG_DIR_TUI"
: > "$LOG_FILE"

if [ "$RESTART" -eq 1 ] && tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi

cleanup() {
  if [ "$CREATED_SESSION" -eq 1 ] && tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
  fi
}

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists; attaching."
  echo "Use --restart to kill it and start bringup_all again."
  if [ -n "${TMUX:-}" ]; then
    tmux switch-client -t "$SESSION"
  else
    tmux attach-session -t "$SESSION"
  fi
  exit 0
fi

log_file_q=$(quote "$LOG_FILE")
ros_log_dir_q=$(quote "$ROS_LOG_DIR_TUI")
voice_regex_q=$(quote "$VOICE_REGEX")
session_q=$(quote "$SESSION")

package_prefix="$(ros2 pkg prefix rover_bringup 2>/dev/null || true)"
setup_file="${ROVER_TUI_SETUP:-}"
if [ -z "$setup_file" ] && [ -n "$package_prefix" ]; then
  setup_file="$(dirname "$package_prefix")/setup.bash"
fi
if [ -z "$setup_file" ] && [ -f "$HOME/colcon_ws/install/setup.bash" ]; then
  setup_file="$HOME/colcon_ws/install/setup.bash"
fi
setup_file_q=$(quote "$setup_file")

launch_cmd="ROS_LOG_DIR=${ros_log_dir_q} ros2 launch rover_bringup bringup_all.launch.py"
for arg in "$@"; do
  launch_cmd+=" $(quote "$arg")"
done

voice_cmd="touch ${log_file_q}; printf 'VOICE: wake_word / stt / tts\\nLog: ${LOG_FILE}\\n\\n'; tail -n +1 -F ${log_file_q} | grep -Ei --line-buffered ${voice_regex_q}"
bringup_inner="printf 'BRINGUP: all other logs\nLog: ${LOG_FILE}\n\n'; if [ -f ${setup_file_q} ]; then source ${setup_file_q}; else echo '[bringup_all_tui] setup.bash not found; continuing with current environment'; fi; ${launch_cmd} 2>&1 | tee -a ${log_file_q} | grep -Eiv --line-buffered ${voice_regex_q}; status=\${PIPESTATUS[0]}; printf '\n[bringup_all_tui] bringup_all exited with code %s\n' \"\$status\"; if [ \"\$status\" -eq 0 ] || [ \"\$status\" -eq 120 ] || [ \"\$status\" -eq 130 ] || [ \"\$status\" -eq 143 ]; then tmux kill-session -t ${session_q}; else printf 'Press Enter to keep this pane open...'; read -r _; fi"
bringup_cmd="bash -lc $(quote "$bringup_inner")"

bringup_pane="$(tmux new-session -d -P -F '#{pane_id}' -s "$SESSION" -n bringup "$bringup_cmd")"
CREATED_SESSION=1
trap cleanup INT TERM EXIT
tmux set-window-option -t "${SESSION}:0" remain-on-exit on >/dev/null
tmux set-window-option -t "${SESSION}:0" pane-border-status top >/dev/null
voice_pane="$(tmux split-window -P -F '#{pane_id}' -v -l 25% -b -t "$bringup_pane" "$voice_cmd")"
tmux select-pane -t "$voice_pane" -T "voice: wake_word/stt/tts"
tmux select-pane -t "$bringup_pane" -T "bringup logs"
tmux select-pane -t "$bringup_pane"

if [ -n "${TMUX:-}" ]; then
  tmux switch-client -t "$SESSION"
else
  tmux attach-session -t "$SESSION"
fi
