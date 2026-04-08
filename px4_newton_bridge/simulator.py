import math
import time

import numpy as np
import warp as wp
from newton.sensors import SensorTiledCamera

import newton

from .builders import BuilderBase
from .logging import logger
from .mavlink_interface import MAVLinkInterface
from .propeller_basic import (
    MotorModel,
    step_motor_model,
    update_body_f,
    update_rotor_states,
)

STABILIZE_VEL_THRESHOLD = 0.01  # m/s
STABILIZE_MIN_STEPS = 10
STABILIZE_MAX_STEPS = 10000


class Simulator:
    def __init__(
        self,
        cfg: dict,
        vehicle_builder: BuilderBase,
    ):
        logger.debug(f"Default device: {wp.get_device()}")

        self.vehicle_builder = vehicle_builder

        self.sim_dt = cfg["physics"]["dt"]
        self.sim_time = 0.0
        self.physics_enabled = cfg["physics"]["enabled"]
        if not self.physics_enabled:
            logger.info("Sensor-only mode: physics disabled")
        self.grpc_server = None
        if cfg["api"]["enabled"]:
            from .grpc_server import GRPCServer

            self.grpc_server = GRPCServer(cfg["api"]["port"])

        self.motor_params = MotorModel()
        self.motor_params.rpm_max = 3800
        self.motor_params.tau = 0.033
        self.motor_params.ct = 0.000003463
        self.motor_params.cd = 0.05
        self.motor_params.dt = 0.004

        self.rpms = wp.zeros(4)
        self.actuator_controls = wp.zeros(4)

        builder = newton.ModelBuilder()
        builder.add_ground_plane()

        self.ref_alt = None
        terrain_cfg = cfg.get("terrain", {})
        if terrain_cfg.get("enabled", False):
            self.ref_alt = self._add_terrain(builder, cfg)

        self.vehicle_builder.build(builder)
        self.model = builder.finalize()
        self.vehicle_builder.model_debug_print(self.model)

        self.solver = newton.solvers.SolverMuJoCo(self.model, njmax=224)

        self.state0 = self.model.state()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state0)
        self.state1 = self.model.state()
        self.control = self.model.control()
        self.contacts = self.model.collide(self.state0)

        logger.debug(f"control dim {self.model.joint_dof_count}")

        # Buffer to store previous body velocities for acceleration computation
        self._body_qd_prev = wp.zeros_like(self.state0.body_qd)

        self.capture()

        self.rtf = cfg["physics"].get("rtf", 0)
        logger.info(f"Real time factor: {self.rtf}")
        self._step_start_time = time.time()

        # Camera sensor (optional, configured per-vehicle)
        self._camera = None
        self._camera_color = None
        self._camera_depth = None
        cam_cfg = vehicle_builder.cfg.get("camera")
        if cam_cfg:
            self._init_camera(cam_cfg)

    @staticmethod
    def _add_terrain(builder: newton.ModelBuilder, cfg: dict) -> float:
        from terrain_fetcher import Terrain

        gps = cfg["sensors"]["gps"]["init"]
        ref_lat, ref_lon = gps["lat"], gps["lon"]

        half_n = cfg["terrain"]["half_lengths"]["north"]
        half_e = cfg["terrain"]["half_lengths"]["east"]

        deg_per_m_lat = 1.0 / 111_000.0
        deg_per_m_lon = deg_per_m_lat / math.cos(math.radians(ref_lat))

        terrain = Terrain(
            ref_lat=ref_lat,
            ref_lon=ref_lon,
            lat_min=ref_lat - half_n * deg_per_m_lat,
            lat_max=ref_lat + half_n * deg_per_m_lat,
            lon_min=ref_lon - half_e * deg_per_m_lon,
            lon_max=ref_lon + half_e * deg_per_m_lon,
        )

        # Warp Texture2D requires 1, 2, or 4 channels; convert RGB -> RGBA
        texture_rgba = np.concatenate(
            [
                terrain.texture_rgb,
                np.full(
                    (
                        *terrain.texture_rgb.shape[:2],
                        1,
                    ),
                    255,
                    dtype=np.uint8,
                ),
            ],
            axis=2,
        )
        mesh = newton.Mesh(
            terrain.vertices,
            terrain.indices.ravel(),
            uvs=terrain.uvs,
            texture=texture_rgba,
            compute_inertia=False,
        )
        builder.add_shape_mesh(body=-1, mesh=mesh)
        logger.info(f"Terrain mesh added: {len(terrain.vertices)} vertices, {len(terrain.indices)} triangles")
        return terrain.ref_alt_wgs84

    def _get_sensor_data(self):
        """TODO: warp kernel to compute sensor data from sim state."""
        pass

    def _init_camera(self, cam_cfg: dict) -> None:
        """Initialize the tiled camera sensor from vehicle camera config."""
        self._cam_width = cam_cfg.get("width", 320)
        self._cam_height = cam_cfg.get("height", 240)
        fov_deg = cam_cfg.get("fov", 90)
        rate = cam_cfg.get("rate", 30)
        self._cam_step_interval = max(1, round(1.0 / (rate * self.sim_dt)))

        # Body-relative camera pose (FRD body frame)
        pos = cam_cfg.get("position", [0.0, 0.0, 0.0])
        rpy_deg = cam_cfg.get("orientation_rpy", [0.0, 0.0, 0.0])
        rpy_rad = [math.radians(a) for a in rpy_deg]

        # Build body-relative transform: RPY -> quaternion (intrinsic XYZ)
        qx = wp.quat_from_axis_angle(wp.vec3(1, 0, 0), rpy_rad[0])
        qy = wp.quat_from_axis_angle(wp.vec3(0, 1, 0), rpy_rad[1])
        qz = wp.quat_from_axis_angle(wp.vec3(0, 0, 1), rpy_rad[2])
        q_body = wp.mul(wp.mul(qz, qy), qx)
        self._cam_body_offset = wp.transform(wp.vec3(*pos), q_body)

        self._camera = SensorTiledCamera(
            model=self.model,
            config=SensorTiledCamera.RenderConfig(enable_textures=True),
        )
        self._camera.utils.create_default_light(enable_shadows=True)

        self._cam_rays = self._camera.utils.compute_pinhole_camera_rays(
            self._cam_width, self._cam_height, math.radians(fov_deg)
        )
        self._camera_color = self._camera.utils.create_color_image_output(self._cam_width, self._cam_height)
        self._camera_depth = self._camera.utils.create_depth_image_output(self._cam_width, self._cam_height)
        self._cam_step_counter = 0
        self._cam_images_dirty = False

        logger.info(
            f"Camera sensor: {self._cam_width}x{self._cam_height} @ {rate} Hz "
            f"(every {self._cam_step_interval} steps), FOV {fov_deg}°"
        )

    def _get_camera_world_transform(self) -> wp.transformf:
        """Compute camera world-space transform from body pose and offset."""
        body_q = self.state0.body_q.numpy()[0]
        body_pos = wp.vec3(float(body_q[0]), float(body_q[1]), float(body_q[2]))
        body_rot = wp.quat(float(body_q[3]), float(body_q[4]), float(body_q[5]), float(body_q[6]))
        body_tf = wp.transform(body_pos, body_rot)
        return wp.transform_multiply(body_tf, self._cam_body_offset)

    def update_camera(self) -> bool:
        """Render camera if due. Returns True if new images are available."""
        if self._camera is None:
            return False

        self._cam_step_counter += 1
        if self._cam_step_counter < self._cam_step_interval:
            return False
        self._cam_step_counter = 0

        cam_tf = self._get_camera_world_transform()
        # camera_transforms shape: (camera_count=1, world_count=1)
        cam_transforms = wp.array([[cam_tf]], dtype=wp.transformf)

        self._camera.update(
            self.state0,
            cam_transforms,
            self._cam_rays,
            color_image=self._camera_color,
            depth_image=self._camera_depth,
        )
        self._cam_images_dirty = True
        return True

    def get_camera_images(self) -> tuple[np.ndarray, np.ndarray] | None:
        """Return (rgb, depth) numpy arrays if new images are available.

        RGB is uint8 (H, W, 3), depth is float32 (H, W) in meters.
        Returns None if no new images since last call.
        """
        if not self._cam_images_dirty:
            return None
        self._cam_images_dirty = False

        # color: (world=1, cam=1, H, W) uint32 RGBA -> (H, W, 3) uint8
        color_np = self._camera_color.numpy()[0, 0]
        rgba = color_np.view(np.uint8).reshape(self._cam_height, self._cam_width, 4)
        rgb = rgba[:, :, :3]

        # depth: (world=1, cam=1, H, W) float32 -> (H, W)
        depth = self._camera_depth.numpy()[0, 0]

        return rgb, depth

    def capture(self):
        """Capture CUDA graph."""
        self.graph = None
        if wp.get_device().is_cuda and wp.is_mempool_enabled(wp.get_device()):
            logger.info("Using CUDA graph")
            if self.physics_enabled:
                with wp.ScopedCapture() as capture:
                    self._simulate_physics()
                    self._get_sensor_data()
            else:
                with wp.ScopedCapture() as capture:
                    self._get_sensor_data()
            self.graph = capture.graph
        else:
            logger.info("CUDA graph not available, using standard simulation")

    def _simulate_physics(self):
        """Run a single physics step - suitable for CUDA graph capture.

        Steps state0 into state1, then copies state1 back to state0 so the
        CUDA graph always operates on the same buffers across replays.
        """
        wp.launch(
            step_motor_model,
            dim=4,
            inputs=(self.actuator_controls, self.motor_params),
            outputs=(self.rpms,),
        )
        wp.launch(
            update_rotor_states,
            dim=4,
            inputs=(self.motor_params, self.rpms),
            outputs=(self.state0.joint_q, self.state0.joint_qd),
        )
        # newton.eval_fk(
        # self.model, self.state0.joint_q, self.state0.joint_qd, self.state0
        # )

        self._body_qd_prev.assign(self.state0.body_qd)
        self.state0.clear_forces()
        wp.launch(
            update_body_f,
            dim=4,
            inputs=(self.motor_params, self.state0.body_q, self.rpms),
            outputs=(self.state0.body_f,),
        )
        self.contacts = self.model.collide(self.state0)
        self.solver.step(self.state0, self.state1, self.control, self.contacts, self.sim_dt)
        self.state0.assign(self.state1)

    def simulate(self, mav: MAVLinkInterface):
        """Full simulation step including MAVLink I/O and physics.

        Implements lockstep synchronization with PX4:
        1. Send current sensor data to PX4
        2. Wait (blocking) for actuator controls from PX4
        3. Run physics with received controls (or apply pose in sensor-only mode)
        """
        if not self.physics_enabled and self.grpc_server is not None:
            mav.gps_fix_type = self.grpc_server.gps_fix_type

            pos = self.grpc_server.pos
            quat = self.grpc_server.quat_xyzw
            omega = self.grpc_server.omega

            # Integrate angular velocity into orientation
            speed = wp.length(omega)
            if speed > 0.0:
                axis = wp.normalize(omega)
                angle = speed * self.sim_dt
                rotation = wp.quat_from_axis_angle(axis, angle)
                quat = wp.mul(rotation, quat)
                self.grpc_server.quat_xyzw = quat

            # Tick ChangeAttTo transition
            transition = self.grpc_server.att_transition
            if transition is not None:
                transition["remaining_time"] -= self.sim_dt
                if transition["remaining_time"] <= 0.0:
                    self.grpc_server.quat_xyzw = transition["target"]
                    quat = transition["target"]
                    self.grpc_server.omega = wp.vec3(0.0, 0.0, 0.0)
                    self.grpc_server.att_transition = None

            joint_q = self.state0.joint_q.numpy()
            joint_q[0:7] = wp.transform(pos, quat)
            self.state0.joint_q.assign(joint_q)

            joint_qd = self.state0.joint_qd.numpy()
            joint_qd[3:6] = omega
            self.state0.joint_qd.assign(joint_qd)

            newton.eval_fk(self.model, self.state0.joint_q, self.state0.joint_qd, self.state0)
            self._body_qd_prev.assign(self.state0.body_qd)

        # Step PX4
        mav._send_sensor_data(self.state0, self._body_qd_prev.numpy(), self.sim_time)

        # Block waiting for actuator controls (lockstep synchronization)
        if not mav.receive_actuator_controls(timeout=2.0):
            raise ConnectionError("PX4 disconnected (no actuator controls received)")

        # Copy actuator controls to GPU (H2D transfer, cannot be in CUDA graph)
        self.actuator_controls.assign(mav.actuator_controls[:4])

        if self.physics_enabled:
            # Run physics (uses CUDA graph if available)
            if self.graph:
                wp.capture_launch(self.graph)
            else:
                self._simulate_physics()

    def stabilize(self):
        """Step physics with zero actuator inputs until the body settles.

        This should be called before wait_for_px4 so PX4 starts receiving sensor data
        at the resting position rather than at the spawn height.
        """
        if not self.physics_enabled:
            return
        for i in range(STABILIZE_MAX_STEPS):
            self._simulate_physics()
            wp.synchronize()
            body_qd = self.state0.body_qd.numpy()
            linear_vel = np.linalg.norm(body_qd[0, :3])

            if i > STABILIZE_MIN_STEPS and linear_vel < STABILIZE_VEL_THRESHOLD:
                logger.info(f"Stabilized after {i + 1} steps (vel={linear_vel:.4f} m/s)")
                return

        logger.warning(f"Stabilization did not converge after {STABILIZE_MAX_STEPS} steps (vel={linear_vel:.4f} m/s)")

    def step(self, mav: MAVLinkInterface):
        # Increment sim_time BEFORE simulate() so sensor data has non-zero timestamps
        self.sim_time += self.sim_dt

        # simulate() handles I/O and physics (with CUDA graph if available)
        self.simulate(mav)

        if self.rtf > 0:
            step_time = time.time() - self._step_start_time
            target_step_time = self.sim_dt / self.rtf
            sleep_time = target_step_time - step_time
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._step_start_time = time.time()
