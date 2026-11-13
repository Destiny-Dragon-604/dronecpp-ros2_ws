#!/usr/bin/env bash
# Start PX4 SITL (x500) in Gazebo Harmonic with the SuaveDisturbance plugin loaded.
#
#   tools/sim.sh                  default world, with the Gazebo window
#   tools/sim.sh baylands         any PX4 world (default, windy, baylands, lawn, walls, ...)
#   tools/sim.sh --headless       no Gazebo window (faster, fine for scripted tests)
#
# Then, in another terminal:  ros2 run suave_launch suave_main
# Disturbances:               type them in suave_main, or use tools/disturb.py
# Stop: type "shutdown" at the pxh> prompt or press Ctrl+C here.

set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/Dev/PX4-Autopilot}"
WORLD="default"
HEADLESS_MODE=""

for arg in "$@"; do
    case "$arg" in
        --headless) HEADLESS_MODE=1 ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) WORLD="$arg" ;;
    esac
done

GZ_ENV="$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"
PLUGIN_DIR="$WS/install/suave_gz_disturbance/lib"
SERVER_CONFIG="$WS/install/suave_gz_disturbance/share/suave_gz_disturbance/server.config"

[ -f "$GZ_ENV" ] || { echo "PX4 SITL is not built. Run: cd $PX4_DIR && make px4_sitl"; exit 1; }
[ -f "$PLUGIN_DIR/libSuaveDisturbance.so" ] || { echo "Disturbance plugin not built. Run: cd $WS && colcon build"; exit 1; }

# Same environment PX4 uses for Gazebo (models, worlds, its plugins), plus ours.
# shellcheck disable=SC1090
source "$GZ_ENV"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GZ_SIM_SYSTEM_PLUGIN_PATH:$PLUGIN_DIR"
export GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG"
export SUAVE_DATA_DIR="${SUAVE_DATA_DIR:-$WS/data}"
export GZ_IP=127.0.0.1   # PX4 talks to Gazebo on localhost; everything else must match
mkdir -p "$SUAVE_DATA_DIR"

WORLD_FILE="$PX4_GZ_WORLDS/$WORLD.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "Unknown world '$WORLD'. Available:"; ls "$PX4_GZ_WORLDS" | sed 's/\.sdf$//; s/^/  /'
    exit 1
fi
if gz topic -l 2>/dev/null | grep -q "^/world/.*/clock"; then
    echo "A Gazebo world is already running. Close it first (or: pkill -f 'gz sim')."
    exit 1
fi

GZ_PIDS=()
cleanup() {
    for pid in "${GZ_PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT

echo "Starting Gazebo world '$WORLD' with the disturbance plugin (data: $SUAVE_DATA_DIR)"
gz sim -r -s -v 2 "$WORLD_FILE" &
GZ_PIDS+=($!)
if [ -z "$HEADLESS_MODE" ]; then
    gz sim -g > /dev/null 2>&1 &
    GZ_PIDS+=($!)
fi

for _ in $(seq 1 60); do
    gz topic -l 2>/dev/null | grep -q "^/world/.*/clock" && break
    sleep 1
done

# PX4 finds the running world, spawns x500_0 into it and starts its bridge.
export PX4_GZ_FOLLOW=1
cd "$PX4_DIR"
make px4_sitl gz_x500
