import argparse

import warp as wp

from . import get_cfg, load_model
from .logging import logger
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

    if cfg["physics"]["force_cpu"]:
        wp.set_device("cpu")

    viewer = Viewer(cfg)
    sim = Simulator(cfg, load_model(args.vehicle))
    viewer.set_model(sim.model)

    sim.stabilize()
    mav = MAVLinkInterface(cfg)
    if sim.ref_alt is not None:
        mav.ref_alt = sim.ref_alt
    sim.sim_time = mav.wait_for_px4(sim.state0, sim._body_qd_prev.numpy())

    try:
        while True:
            sim.step(mav)
            sim.update_camera()
            viewer.begin_frame(sim.sim_time)
            viewer.log_state(sim.state0)
            images = sim.get_camera_images()
            if images is not None:
                viewer.log_camera_images(*images)
            viewer.end_frame()
    except ConnectionError as e:
        logger.info(e)

    viewer.close()


if __name__ == "__main__":
    main()
