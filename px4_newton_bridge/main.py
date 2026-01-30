import argparse

from .models import load_model
from .vehicle import Vehicle
from .viewer import Viewer


def main():
    parser = argparse.ArgumentParser(
        description="PX4 SITL bridge for Newton physics engine",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--model",
        type=str,
        default="default",
        help="Vehicle model config to simulate (name of YAML file in models/configs/).",
    )
    args = parser.parse_args()

    vehicle_model = load_model(args.model)
    vehicle = Vehicle(vehicle_model)
    viewer = Viewer(sim_dt=vehicle.sim_dt)
    viewer.set_model(vehicle.model)

    vehicle.wait_for_px4()

    while viewer.is_running():
        if not viewer.is_paused():
            vehicle.step()
        viewer.begin_frame(vehicle.sim_time)
        viewer.log_state(vehicle.state0)
        viewer.end_frame()

    viewer.close()


if __name__ == "__main__":
    main()
