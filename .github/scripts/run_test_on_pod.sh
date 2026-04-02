#!/usr/bin/env bash
#
# Run the Newton SITL takeoff/land test natively on a RunPod GPU pod.
#
# Usage: run_test_on_pod.sh <commit_sha>
#
# This script is piped via SSH from the GitHub Actions runner.
# It expects the px4-sitl-newton Docker image's dependencies
# (cmake, ninja, g++, uv, Python) to be pre-installed.
#
set -euo pipefail

COMMIT_SHA="${1:?Usage: $0 <commit_sha>}"
PX4_REPO_URL="https://github.com/PX4/PX4-Autopilot.git"

BRIDGE_SUBMODULE_PATH="Tools/simulation/newton/px4-newton-bridge"
PX4_DIR="/workspace/px4"
PX4_TARGET="px4_sitl newton_astro_max"
PX4_STARTUP_WAIT=20
PX4_LOG="/tmp/px4_console.log"

cleanup() {
    echo "[ci] Cleaning up..."
    pkill -9 -f 'make px4_sitl' 2>/dev/null || true
    pkill -9 -f ninja 2>/dev/null || true
    pkill -9 -f 'bin/px4' 2>/dev/null || true
    pkill -9 -f simulator_mavlink 2>/dev/null || true
    pkill -9 -f px4_newton_bridge 2>/dev/null || true
    rm -f /tmp/px4_sitl_lockfile-* 2>/dev/null || true
    sleep 2
}
trap cleanup EXIT

echo "============================================================"
echo "[ci] GPU SITL Integration Test"
echo "[ci] Commit: $COMMIT_SHA"
echo "[ci] GPU:    $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo 'unknown')"
echo "============================================================"

# Rewrite SSH URLs to HTTPS so public submodules clone without SSH keys
git config --global url."https://github.com/".insteadOf "git@github.com:"

# --- 1. Clone PX4 repo ---
echo "[ci] Cloning PX4 firmware..."
git clone --depth 1 "$PX4_REPO_URL" "$PX4_DIR"
cd "$PX4_DIR"

# Initialize only the bridge submodule
git submodule update --init --depth 1 "$BRIDGE_SUBMODULE_PATH"

# Checkout the exact bridge commit being tested
cd "$PX4_DIR/$BRIDGE_SUBMODULE_PATH"
git fetch origin "$COMMIT_SHA"
git checkout "$COMMIT_SHA"
git submodule update --init --recursive

# --- 2. Install Python environment for Newton bridge ---
echo "[ci] Setting up Python environment..."
cd "$PX4_DIR/$BRIDGE_SUBMODULE_PATH"
uv sync

# --- 3. Start PX4 SITL in background ---
echo "[ci] Starting PX4 SITL ($PX4_TARGET)..."
cleanup
cd "$PX4_DIR"
make $PX4_TARGET > "$PX4_LOG" 2>&1 &
PX4_PID=$!

echo "[ci] Waiting ${PX4_STARTUP_WAIT}s for PX4 + Newton bridge to start..."
sleep "$PX4_STARTUP_WAIT"

if ! kill -0 "$PX4_PID" 2>/dev/null; then
    echo "[ci] ERROR: PX4 process died during startup"
    echo "[ci] PX4 log:"
    strings "$PX4_LOG" | tail -50
    exit 1
fi

# --- 4. Run the test ---
echo "[ci] Running takeoff/land test..."
set +e
cd "$PX4_DIR/$BRIDGE_SUBMODULE_PATH"
uv run --extra test python tests/test_takeoff_land.py
TEST_EXIT=$?
set -e

# --- 5. Show PX4 console output ---
echo ""
echo "============================================================"
echo "PX4 Console Output"
echo "============================================================"
strings "$PX4_LOG" | grep -vE '^pxh>\s*$' | grep -vE '^(\s|$)' || true

exit $TEST_EXIT
