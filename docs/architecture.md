# Architecture

## Process tree

```
make px4_sitl_newton
  cmake -E env ... bin/px4
    px4-rc.newtonsim (sourced by PX4 init)
      newton_run.sh &
        setsid uv run rerun &        (local only, own session)
        uv run python3 -m px4_newton_bridge.main
      simulator_mavlink start -c 4560
      rerun_logger start -s newton
```

## Communication

```
PX4 (simulator_mavlink) <──TCP:4560──> Bridge (MAVLinkInterface)
PX4 (rerun_logger)      ──gRPC:9876──> Rerun viewer
Bridge (ViewerRerun)     ──gRPC:9876──> Rerun viewer
```

PX4 and the bridge communicate via MAVLink over TCP in lockstep: the bridge
sends sensor data, then blocks waiting for actuator controls (with a 2-second
timeout for disconnect detection).

Both the PX4 `rerun_logger` module and the bridge log to the Rerun viewer via
gRPC on port 9876.

## Startup

Running `make px4_sitl_newton newton_<vehicle>` starts PX4, which sources
`px4-rc.newtonsim` during init. This script:

1. Kills any leftover bridge process from a previous session. This is necessary
   because a stale bridge still listening on port 4560 would cause the new PX4
   instance to connect to it instead of waiting for the fresh bridge.
2. Launches `newton_run.sh` in the background, passing `PX4_PID=$$` so the
   script knows which PX4 instance started it.
3. Starts `simulator_mavlink` in TCP client mode (PX4 retries connecting to the
   bridge on port 4560 until the bridge is ready).
4. Starts `rerun_logger`, which connects to the Rerun viewer on port 9876.

`newton_run.sh` then:

1. Installs Python dependencies via `uv sync`.
2. Starts the Rerun viewer in its own session via `setsid` (local only — skipped
   in Docker since the viewer runs on the host via `run_docker.sh`). The separate
   session prevents Ctrl-C from killing the viewer along with PX4, so it persists
   across sim restarts.
3. Launches the bridge (`px4_newton_bridge.main`), with stdout/stderr redirected
   to `startup.log`.

The bridge initializes Warp (CUDA kernel compilation), builds the physics model,
and stabilizes the simulation. It then listens for PX4 on TCP port 4560 via
`wait_for_px4()`, which has a 2-second timeout. Once PX4 connects and starts
sending actuator controls, lockstep simulation begins.

### Console output

The bridge's stdout/stderr is redirected to `startup.log` so that Warp kernel
compilation messages don't pollute the PX4 console. If PX4 exits before the
bridge connects (e.g. Ctrl-C during init), the bridge's output won't overwrite
the terminal prompt. Bridge logs (warnings, errors, status) go to the Rerun
viewer via a custom Python logging handler.

## Shutdown

### Docker

In Docker, all processes (PX4, bridge, Rerun logger) run inside a single
container started by `docker compose run`. Ctrl-C sends SIGINT to the container,
which terminates all processes at once. The Rerun viewer runs on the host and is
unaffected.

### Local

PX4, the bridge, and `newton_run.sh` all run in the same terminal but as
separate processes. The bridge runs as a background process (launched via
`./newton_run.sh &` in `px4-rc.newtonsim`), so it does **not** receive SIGINT
from Ctrl-C.

#### Ctrl-C after MAVLink connection is established

1. Ctrl-C sends SIGINT to PX4 (foreground process).
2. PX4 shuts down, closing the TCP connection on port 4560.
3. The bridge detects the closed connection within its 2-second MAVLink timeout
   and raises `ConnectionError`.
4. The bridge exits, `viewer.close()` disconnects from Rerun.
5. `newton_run.sh` sends SIGTERM to the specific PX4 instance that launched it
   (via `PX4_PID`, already dead at this point). The Rerun viewer stays open.

#### Ctrl-C before MAVLink connection (during Warp init)

1. Ctrl-C sends SIGINT to PX4.
2. PX4 shuts down, but the bridge is still initializing. Warp kernel compilation
   runs in native CUDA code that blocks Python signal handling, so the bridge
   cannot respond to signals during this phase.
3. The bridge finishes init and enters `wait_for_px4()`, which has a 2-second
   timeout. Since PX4 is dead, the timeout expires and `ConnectionError` is
   raised.
4. The bridge exits cleanly. `newton_run.sh` sends SIGTERM to its specific PX4
   instance (via `PX4_PID`). The Rerun viewer stays open.

!!! note
    `newton_run.sh` uses the `PX4_PID` environment variable (set by
    `px4-rc.newtonsim`) to kill only its own PX4 instance, not any other
    `bin/px4` process. This prevents a stale `newton_run.sh` from killing a
    newly started PX4 session.

## Local vs Docker

| Aspect | Local | Docker |
|---|---|---|
| Rerun viewer | Started by `newton_run.sh` | Started by `run_docker.sh` on the host |
| Warp kernel cache | `~/.cache/warp` | Docker volume (`warp-cache`) |
| Python venv | `.venv/` | Docker volume (`venv`) at `/var/cache/venv` |
| Rerun gRPC errors | Suppressed by redirecting bridge output to `startup.log` | Suppressed via `RUST_LOG` in `docker-compose.yaml` |
