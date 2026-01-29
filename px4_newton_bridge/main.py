import argparse

from .drone import Drone
from .viewer import Viewer


def main():
    parser = argparse.ArgumentParser(
        description="PX4 SITL bridge for Newton physics engine",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--platform",
        type=str,
        default="astro",
        help="Which drone platform to simulate.",
    )
    args = parser.parse_args()

    drone = Drone(args.platform)
    viewer = Viewer(sim_dt=drone.sim_dt)
    viewer.set_model(drone.model)

    drone.wait_for_px4()

    while viewer.is_running():
        if not viewer.is_paused():
            drone.step()
        viewer.begin_frame(drone.sim_time)
        viewer.log_state(drone.state0)
        viewer.end_frame()

    viewer.close()


if __name__ == "__main__":
    main()
