#!/usr/bin/env bash
#
# Start PX4 SITL with the Newton bridge, run all px4_sitl_test_* tests on all vehicles.
# SITL is restarted fresh before each test. Each test runs on every vehicle.
#
# Works both locally (from the repo root) and in CI.
#
# Usage: scripts/run_px4_sitl_tests.sh [px4_dir]
#   px4_dir: Path to PX4-Autopilot root (default: auto-detect from submodule path)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_DIR="${1:-$(cd "$BRIDGE_DIR/../../../.." && pwd)}"
PX4_STARTUP_WAIT=20
PX4_LOG="/tmp/px4_console.log"
VEHICLES_DIR="$BRIDGE_DIR/px4_newton_bridge/vehicles"

cleanup() {
    pkill -9 -f 'make px4_sitl' 2>/dev/null || true
    pkill -9 -f ninja 2>/dev/null || true
    pkill -9 -f 'bin/px4' 2>/dev/null || true
    pkill -9 -f simulator_mavlink 2>/dev/null || true
    pkill -9 -f px4_newton_bridge 2>/dev/null || true
    rm -f /tmp/px4_sitl_lockfile-* 2>/dev/null || true
    sleep 2
}
trap cleanup EXIT

start_sitl() {
    local vehicle="$1"
    cleanup
    cd "$PX4_DIR"
    make px4_sitl_newton "newton_$vehicle" > "$PX4_LOG" 2>&1 &
    PX4_PID=$!

    echo "[test] Waiting ${PX4_STARTUP_WAIT}s for PX4 + Newton bridge to start..."
    sleep "$PX4_STARTUP_WAIT"

    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        echo "[test] ERROR: PX4 process died during startup"
        echo "[test] PX4 log:"
        strings "$PX4_LOG" | tail -50
        return 1
    fi
}

# Discover vehicles
VEHICLES=()
for v in "$VEHICLES_DIR"/*/; do
    [ -d "$v" ] || continue
    VEHICLES+=("$(basename "$v")")
done

# Discover tests
TESTS=()
for t in "$BRIDGE_DIR"/tests/px4_sitl_test_*.py; do
    [ -f "$t" ] || continue
    TESTS+=("$t")
done

echo "============================================================"
echo "[test] PX4 SITL Integration Tests"
echo "[test] PX4 dir:  $PX4_DIR"
echo "[test] Vehicles: ${VEHICLES[*]}"
echo "[test] Tests:    ${#TESTS[@]}"
echo "[test] GPU:      $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo 'none')"
echo "============================================================"

# Install test dependencies
cd "$BRIDGE_DIR"
uv sync --extra test

# Run each test on each vehicle
FAILED=0
for vehicle in "${VEHICLES[@]}"; do
    for test in "${TESTS[@]}"; do
        TEST_NAME="$(basename "$test")"

        echo ""
        echo "[test] --- $TEST_NAME ($vehicle) ---"
        echo "[test] Starting PX4 SITL (newton_$vehicle)..."
        if ! start_sitl "$vehicle"; then
            FAILED=1
            continue
        fi

        echo "[test] Running $TEST_NAME..."
        set +e
        cd "$BRIDGE_DIR"
        uv run python "$test"
        EXIT=$?
        set -e

        if [ "$EXIT" -ne 0 ]; then
            echo "[test] FAIL: $TEST_NAME ($vehicle) (exit code $EXIT)"
            FAILED=1
        fi

        echo "[test] Stopping SITL..."
        cleanup
    done
done

# Show PX4 console output from last run
echo ""
echo "============================================================"
echo "PX4 Console Output (last run)"
echo "============================================================"
strings "$PX4_LOG" | grep -vE '^pxh>\s*$' | grep -vE '^(\s|$)' || true

exit $FAILED
