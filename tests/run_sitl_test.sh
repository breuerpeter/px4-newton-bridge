#!/usr/bin/env bash
#
# Run the takeoff/land SITL integration test.
#
# 1. Ensures the px4-sitl-newton and px4-sitl-newton-test containers are running
# 2. Launches PX4 SITL (newton_astro_max) inside the sim container
# 3. Builds and runs the MAVSDK C++ test in the test container
# 4. Cleans up the PX4 process
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_DOCKER_DIR="$BRIDGE_DIR/../../../../docker"
SIM_CONTAINER="px4-sitl-newton"
TEST_CONTAINER="px4-sitl-newton-test"
PX4_TARGET="px4_sitl newton_astro_max"
PX4_LOG="/tmp/px4_console.log"

# Time to wait for PX4 to start (wall-clock seconds)
PX4_STARTUP_WAIT=15

cleanup() {
    echo "[test] Cleaning up PX4 process..."
    # Kill the entire process tree: make, ninja, cmake, px4, bridge, uv, simulator_mavlink
    docker exec "$SIM_CONTAINER" bash -c \
        "pkill -9 -f 'make px4_sitl' ; \
         pkill -9 -f 'ninja' ; \
         pkill -9 -f 'bin/px4' ; \
         pkill -9 -f 'simulator_mavlink' ; \
         pkill -9 -f 'px4_newton_bridge' ; \
         pkill -9 -f 'uv run.*px4_newton_bridge' ; \
         rm -f /tmp/px4_sitl_lockfile-*" 2>/dev/null || true
    sleep 2
}

on_signal() {
    # Ignore further signals so cleanup's docker exec isn't interrupted
    trap '' INT TERM EXIT
    cleanup
    exit 1
}

trap on_signal INT TERM
trap cleanup EXIT

# --- 1. Ensure both containers are running ---
echo "[test] Ensuring Docker containers are running..."
(cd "$PX4_DOCKER_DIR" && docker compose up "$SIM_CONTAINER" "$TEST_CONTAINER" -d --wait 2>&1) || {
    echo "[test] ERROR: Failed to start containers"
    exit 1
}

# Kill any previous PX4 instance in the sim container
cleanup

# --- 2. Build the C++ test binary (cached after first build) ---
echo "[test] Building test binary..."
docker exec "$TEST_CONTAINER" bash -c \
    "cmake -B /tmp/build -S /tests \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DCMAKE_C_COMPILER=clang \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/build -j\$(nproc)" || {
    echo "[test] ERROR: Build failed"
    exit 1
}

# --- 3. Start PX4 SITL in background ---
echo "[test] Starting PX4 SITL ($PX4_TARGET) in container..."
docker exec -d "$SIM_CONTAINER" bash -c "cd /px4 && make $PX4_TARGET >$PX4_LOG 2>&1"

echo "[test] Waiting ${PX4_STARTUP_WAIT}s for PX4 + bridge to start..."
sleep "$PX4_STARTUP_WAIT"

# --- 4. Run the test ---
echo "[test] Running takeoff/land test..."
set +e
docker exec "$TEST_CONTAINER" /tmp/build/test_takeoff_land
TEST_EXIT=$?
set -e

# Show PX4 console output (filter out binary log data and empty prompts)
echo ""
echo "============================================================"
echo "PX4 Console Output"
echo "============================================================"
docker exec "$SIM_CONTAINER" bash -c "strings $PX4_LOG | grep -vE '^pxh>\\s*$' | grep -vE '^(\\s|$)'" 2>/dev/null || true

exit $TEST_EXIT
