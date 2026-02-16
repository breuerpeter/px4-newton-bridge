import argparse

import warp as wp

from . import get_cfg, load_model
from .mavlink_interface import MAVLinkInterface
from .simulator import Simulator
from .viewer import Viewer


def main():
    parser = argparse.ArgumentParser(
        description="PX4 SITL bridge for the Newton physics engine",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--vehicle",
        type=str,
        default="quad_x",
        help="Vehicle to simulate (name of YAML file in vehicles/).",
    )
    args = parser.parse_args()

    cfg = get_cfg()
    viewer = Viewer(cfg)
    mav = MAVLinkInterface(cfg)
    sim = Simulator(cfg, mav, load_model(args.vehicle))
    viewer.set_model(sim.model)

    sim.sim_time = mav.wait_for_px4(sim.state0, sim._body_qd_prev.numpy())

    while viewer.is_running():
        if not viewer.is_paused():
            sim.step()
        viewer.begin_frame(sim.sim_time)
        viewer.log_state(sim.state0)
        viewer.end_frame()
        wp.synchronize()

    viewer.close()


if __name__ == "__main__":
    main()
