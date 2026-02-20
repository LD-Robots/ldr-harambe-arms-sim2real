#!/usr/bin/env bash
#
# One-command launcher for Isaac Sim + dual arm robot.
#
# Generates the URDF with system Python (ROS2 xacro), then starts
# Isaac Sim with the robot pre-configured and ROS2 bridge ready.
#
# Usage:
#     ./run_isaac_sim.sh [--headless] [--save-usd FILE] [--load-usd FILE]
#
# Prerequisites:
#     - ROS2 Jazzy workspace built:  colcon build
#     - Isaac Sim 6.0 built from source
#     - Set ISAAC_SIM_PATH if not ~/isaacsim/_build/linux-x86_64/release
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URDF_PATH="/tmp/dual_arm_isaac.urdf"

# Isaac Sim path — override with ISAAC_SIM_PATH env var
ISAAC_SIM="${ISAAC_SIM_PATH:-$HOME/isaacsim/_build/linux-x86_64/release}"

if [ ! -f "$ISAAC_SIM/python.sh" ]; then
    echo "[ERROR] Isaac Sim not found at: $ISAAC_SIM"
    echo "        Set ISAAC_SIM_PATH to your Isaac Sim build/release directory."
    exit 1
fi

# --- Step 1: Source ROS2 workspace and generate URDF ---
# Find the workspace install dir (go up from src/simulation/dual_arm_isaac/scripts)
WS_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

if [ -f "$WS_ROOT/install/setup.bash" ]; then
    echo "[INFO] Sourcing ROS2 workspace: $WS_ROOT/install/setup.bash"
    source "$WS_ROOT/install/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "[WARN] Workspace install not found, sourcing /opt/ros/jazzy/setup.bash"
    source /opt/ros/jazzy/setup.bash
else
    echo "[ERROR] Could not find ROS2 setup.bash"
    exit 1
fi

# Check if --load-usd is passed (skip URDF generation)
SKIP_URDF=false
for arg in "$@"; do
    if [ "$arg" = "--load-usd" ]; then
        SKIP_URDF=true
        break
    fi
done

if [ "$SKIP_URDF" = false ]; then
    echo "[INFO] Generating URDF with absolute mesh paths..."
    python3 "$SCRIPT_DIR/generate_urdf.py" --output "$URDF_PATH"
fi

# --- Step 2: Set up ROS2 env for Isaac Sim's internal ROS2 bridge ---
# Isaac Sim 6.0 bundles its own ROS2 Jazzy libraries (Python 3.11 compatible).
# These env vars tell the bridge extension to use them.
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Unset system ROS2 Python paths that conflict with Isaac Sim's Python 3.11
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH="${ISAAC_SIM}/exts/isaacsim.ros2.bridge/jazzy/lib:${LD_LIBRARY_PATH:-}"

echo "[INFO] Starting Isaac Sim..."
echo "[INFO] After Isaac Sim loads, open another terminal and run:"
echo "         ros2 launch dual_arm_isaac isaac_sim.launch.py"
echo "       Then MoveIt:"
echo "         ros2 launch dual_arm_moveit_config move_group.launch.py"
echo ""

if [ "$SKIP_URDF" = true ]; then
    exec "$ISAAC_SIM/python.sh" "$SCRIPT_DIR/launch_isaac_sim.py" "$@"
else
    exec "$ISAAC_SIM/python.sh" "$SCRIPT_DIR/launch_isaac_sim.py" \
        --urdf-path "$URDF_PATH" "$@"
fi
