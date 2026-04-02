"""
SITL integration test: arm, takeoff, hover, land.

Connects to PX4 via MAVSDK on the offboard UDP port (14540),
commands a takeoff/land sequence while monitoring stability,
and reports real-time factor (RTF) and simulation speed.

Expects PX4 SITL + Newton bridge to already be running.
"""

import asyncio
import sys
import time

from mavsdk import System
from mavsdk.action import ActionError

# Configuration
TAKEOFF_ALT_M = 5.0
MAX_TILT_DEG = 60.0
HOVER_DURATION_S = 5.0
AUTOPILOT_TIMEOUT_S = 60.0
WALL_TIMEOUT_S = 120.0
SIM_PHYSICS_HZ = 250.0  # Newton bridge sim_dt = 0.004 s


async def main():
    conn_url = sys.argv[1] if len(sys.argv) > 1 else "udp://:14540"

    print("=" * 60)
    print("PX4 Newton SITL -- Takeoff / Land Integration Test")
    print("=" * 60)
    print(f"Connecting to {conn_url} ...")

    drone = System()
    await drone.connect(system_address=conn_url)

    # Wait for autopilot
    print("Waiting for PX4 autopilot ...")
    start = time.monotonic()
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("  Autopilot connected")
            break
        if time.monotonic() - start > AUTOPILOT_TIMEOUT_S:
            print("FAIL: No autopilot found within timeout")
            return 1

    # Wait for home position
    print("Waiting for vehicle to be ready ...")
    start = time.monotonic()
    async for health in drone.telemetry.health():
        if health.is_home_position_ok:
            print("  Home position set")
            break
        if time.monotonic() - start > WALL_TIMEOUT_S:
            print("FAIL: Home position not set within timeout")
            return 1

    # Track attitude for tilt check and RTF measurement
    tilt_fail = False
    attitude_count = 0
    last_roll = 0.0
    last_pitch = 0.0

    async def monitor_attitude():
        nonlocal tilt_fail, attitude_count, last_roll, last_pitch
        async for euler in drone.telemetry.attitude_euler():
            last_roll = euler.roll_deg
            last_pitch = euler.pitch_deg
            attitude_count += 1
            if abs(euler.roll_deg) > MAX_TILT_DEG or abs(euler.pitch_deg) > MAX_TILT_DEG:
                tilt_fail = True

    attitude_task = asyncio.create_task(monitor_attitude())

    # Track altitude
    alt_m = 0.0

    async def monitor_position():
        nonlocal alt_m
        async for pos in drone.telemetry.position():
            alt_m = pos.relative_altitude_m

    position_task = asyncio.create_task(monitor_position())

    # Begin timed section for RTF measurement
    wall_start = time.monotonic()
    att_start = attitude_count

    def check(phase: str) -> bool:
        if tilt_fail:
            print(f"FAIL: Excessive tilt during {phase} (roll={last_roll:.1f} pitch={last_pitch:.1f})")
            return False
        if time.monotonic() - wall_start > WALL_TIMEOUT_S:
            print(f"FAIL: Wall-clock timeout during {phase}")
            return False
        return True

    # Arm (retry until PX4's commander allows it)
    print("Arming ...")
    while True:
        try:
            await drone.action.arm()
            break
        except ActionError:
            if time.monotonic() - wall_start > WALL_TIMEOUT_S:
                print("FAIL: Arm failed after timeout")
                return 1
            await asyncio.sleep(1)

    # Takeoff
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
    print(f"Taking off to {TAKEOFF_ALT_M} m ...")
    try:
        await drone.action.takeoff()
    except ActionError as e:
        print(f"FAIL: Takeoff failed: {e}")
        return 1

    # Wait until takeoff altitude reached
    while alt_m < TAKEOFF_ALT_M * 0.6:
        if not check("takeoff"):
            return 1
        await asyncio.sleep(0.05)
    print(f"  Takeoff altitude reached: {alt_m:.1f} m")

    # Hover
    print("Hovering ...")
    hover_start = time.monotonic()
    while time.monotonic() - hover_start < HOVER_DURATION_S:
        if not check("hover"):
            return 1
        await asyncio.sleep(0.05)
    print("  Hover complete")

    # Land
    print("Landing ...")
    try:
        await drone.action.land()
    except ActionError as e:
        print(f"FAIL: Land failed: {e}")
        return 1

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
        if not check("landing"):
            return 1

    print(f"  Landed (alt={alt_m:.1f} m)")

    # Wait for auto-disarm
    await asyncio.sleep(3)

    # End timed section
    wall_end = time.monotonic()
    att_end = attitude_count

    wall_s = wall_end - wall_start
    att_total = att_end - att_start
    attitude_hz = 50.0  # PX4 SITL default
    sim_s = att_total / attitude_hz
    rtf = sim_s / wall_s
    sim_fps = (sim_s * SIM_PHYSICS_HZ) / wall_s

    print(f"\n{'-' * 60}")
    print("RESULT: PASS\n")
    print("Speed Statistics:")
    print(f"  Real-time factor (RTF) : {rtf:.2f}x")
    print(f"  Sim FPS (250 Hz basis) : {int(sim_fps)} steps/s")
    print(f"  Sim time               : {sim_s:.1f} s")
    print(f"  Wall time              : {wall_s:.1f} s")
    print(f"  ATTITUDE messages      : {att_total}")
    print(f"  ATTITUDE msg rate      : {att_total / wall_s:.1f} Hz")
    print(f"{'-' * 60}")

    # Cleanup
    attitude_task.cancel()
    position_task.cancel()

    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
