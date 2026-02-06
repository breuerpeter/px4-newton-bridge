import argparse

from .models import load_model
from .simulator import Simulator
from .viewer import Viewer


def main():
    parser = argparse.ArgumentParser(
        description="PX4 SITL bridge for Newton physics engine",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--vehicle",
        type=str,
        default="quad_x",
        help="Vehicle to simulate (name of YAML file in vehicles/).",
    )
    args = parser.parse_args()

    vehicle_model = load_model(args.vehicle)
    sim = Simulator(vehicle_model)
    viewer = Viewer(sim_dt=sim.sim_dt)
    viewer.set_model(sim.model)

    sim.wait_for_px4()

    while viewer.is_running():
        if not viewer.is_paused():
            sim.step()
        viewer.begin_frame(sim.sim_time)
        viewer.log_state(sim.state0)
        viewer.end_frame()

    viewer.close()


if __name__ == "__main__":
    main()
