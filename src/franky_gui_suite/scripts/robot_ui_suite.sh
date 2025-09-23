#!/usr/bin/env bash
# Wrapper to launch the Robot UI in a consistent environment.
# - Sources ROS (Noetic) if available
# - Sources your catkin workspace
# - Activates the desired conda env (default: franka_conda)
# - Executes the UI Python script
# All comments and logs are in English.

set -Eeuo pipefail

# ---------- Logging helpers ----------
log()  { echo "[robot-ui] $*" >&2; }
warn() { echo "[robot-ui][warn] $*" >&2; }
die()  { echo "[robot-ui][error] $*" >&2; exit 1; }

# ---------- 1) Source ROS (Noetic) ----------
if [ -f "/opt/ros/noetic/setup.bash" ]; then
  # Load ROS environment (catkin, rospack, etc.)
  # Using 'source' ensures environment variables like ROS_PACKAGE_PATH are available.
  source /opt/ros/noetic/setup.bash
  log "ROS Noetic environment sourced."
else
  warn "ROS Noetic not found at /opt/ros/noetic. Continuing without explicit ROS setup."
fi

# ---------- 2) Source your catkin workspace ----------
# Adjust this path if your workspace differs.
WS_SETUP="$HOME/CampUsers/Pei/open_ws/devel/setup.bash"
if [ -f "$WS_SETUP" ]; then
  # Load your overlay workspace so packages like 'franky_gui_suite' are visible.
  source "$WS_SETUP"
  log "Catkin workspace sourced: $WS_SETUP"
else
  warn "Workspace setup not found: $WS_SETUP"
fi

# ---------- 3) Prepare conda (without requiring an interactive login shell) ----------
# We try common installation paths, then fall back to 'conda info --base' if available.
CONDASH=""
if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
  CONDASH="$HOME/miniconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
  CONDASH="$HOME/anaconda3/etc/profile.d/conda.sh"
elif command -v conda >/dev/null 2>&1; then
  # This covers cases where conda was installed elsewhere but is on PATH.
  CONDABASE="$(conda info --base 2>/dev/null || true)"
  if [ -n "${CONDABASE:-}" ] && [ -f "$CONDABASE/etc/profile.d/conda.sh" ]; then
    CONDASH="$CONDABASE/etc/profile.d/conda.sh"
  fi
fi

if [ -n "$CONDASH" ]; then
  # shellcheck source=/dev/null
  source "$CONDASH"
  log "Conda sh loaded: $CONDASH"
else
  warn "Conda init script not found; will attempt to continue without conda."
fi

# Choose which conda env to activate (can be overridden via environment variable).
CONDA_ENV_NAME="${CONDA_ENV_NAME:-franka_conda}"

# ---------- 4) Activate conda env (best-effort) ----------
if command -v conda >/dev/null 2>&1; then
  if conda activate "$CONDA_ENV_NAME" 2>/dev/null; then
    log "Conda env activated: $CONDA_ENV_NAME"
  else
    warn "Failed to activate conda env '$CONDA_ENV_NAME'; using system python."
  fi
else
  warn "Conda command not found; using system python."
fi

# ---------- 5) Locate the Python UI script ----------
# Primary strategy: assume the .sh and .py are installed in the same directory by catkin.
SELF_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
UI_SCRIPT="$SELF_DIR/robot_ui_suite.py"

# Fallback: resolve via rospack if the sibling script is missing.
if [ ! -f "$UI_SCRIPT" ]; then
  if command -v rospack >/dev/null 2>&1; then
    PKG_PATH="$(rospack find franky_gui_suite 2>/dev/null || true)"
    if [ -n "$PKG_PATH" ] && [ -f "$PKG_PATH/scripts/robot_ui_suite.py" ]; then
      UI_SCRIPT="$PKG_PATH/scripts/robot_ui_suite.py"
    fi
  fi
fi

# Final check: error if we still cannot locate the UI script.
[ -f "$UI_SCRIPT" ] || die "UI script not found. Expected at '$SELF_DIR/robot_ui_suite.py' or '<pkg>/scripts/robot_ui_suite.py'."

# ---------- 6) Choose python and launch ----------
PYBIN="${PYBIN:-python3}"
if ! command -v "$PYBIN" >/dev/null 2>&1; then
  die "Python interpreter '$PYBIN' not found."
fi

log "Launching UI with: $PYBIN $UI_SCRIPT"
exec "$PYBIN" "$UI_SCRIPT"
