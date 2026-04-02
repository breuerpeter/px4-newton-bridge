#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$SCRIPT_DIR/docker"
PX4_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

if [[ $# -eq 0 ]]; then
    echo "Usage: ./run_docker.sh <vehicle>"
    echo "Example: ./run_docker.sh astro_max"
    exit 1
fi

vehicle="$1"

# Start the Rerun viewer on the host (if not already running)
if ! command -v rerun >/dev/null 2>&1; then
    echo "Rerun viewer not found"
elif pgrep -x rerun >/dev/null 2>&1; then
    echo "Rerun viewer already running"
else
    echo "Starting Rerun viewer..."
    rerun >/dev/null 2>&1 &
    disown
fi

compose_args=(-f "$DOCKER_DIR/docker-compose.yaml")
if nvidia-smi &>/dev/null; then
    compose_args+=(-f "$DOCKER_DIR/docker-compose.gpu.yaml")
fi

cd "$DOCKER_DIR" && PX4_DIR="$PX4_DIR" \
    docker compose "${compose_args[@]}" \
    run --rm --user "$(id -u):$(id -g)" \
    px4-sitl-newton make px4_sitl_newton "newton_$vehicle"
