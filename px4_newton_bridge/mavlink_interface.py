import math
import os
import random
import time
from typing import cast

import newton
import numpy as np
import warp as wp

# Set MAVLink dialect before importing mavutil
os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "common"

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

from .logging import logger


class MAVLinkInterface:
    """Handles MAVLink communication with a PX4 SITL instance."""

    _hil_act_msg = mavutil.mavlink.MAVLink_hil_actuator_controls_message
    NUM_ACTUATOR_CHANNELS = _hil_act_msg.array_lengths[
        _hil_act_msg.ordered_fieldnames.index("controls")
    ]

    def __init__(
        self, cfg: dict, ip: str = "0.0.0.0", sysid: int = 1, compid: int = 200
    ):
        self.sim_dt = cfg["sim"]["dt"]
        self.rng = random.Random(42)  # deterministic sensor noise
        conn_string = f"tcpin:{ip}:4560"
        logger.info(f"Waiting for PX4 connection on {conn_string}...")
        self.mav = cast(
            mavutil.mavtcpin,
            mavutil.mavlink_connection(
                conn_string,
                source_system=sysid,
                source_component=compid,
            ),
        )
        self.proto: mavlink2.MAVLink = self.mav.mav
        self.proto.srcSystem = sysid
        self.proto.srcComponent = compid
        self.mav.target_system = 1
        self.mav.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

        logger.info(
            f"Using MAVLink source sysid/compid {self.proto.srcSystem}/{self.proto.srcComponent} "
            f"and target sysid/compid {self.mav.target_system}/{self.mav.target_component}"
        )

        self.actuator_controls = [0.0] * self.NUM_ACTUATOR_CHANNELS

        self.last_hil_sensor_time = 0.0
        self.last_hil_gps_time = 0.0

        self.hil_sensor_interval = 1.0 / 250.0  # [s]
        self.hil_gps_interval = 1.0 / 10.0  # [s]

    def receive_actuator_controls(self, timeout: float | None = None) -> bool:
        """
        Receive actuator controls (HIL_ACTUATOR_CONTROLS) message from PX4.

        Blocks until actuator controls are received or timeout expires.
        This is required for proper lockstep synchronization with PX4.

        Args:
            timeout: Maximum time to wait for message in seconds.

        Returns:
            True if actuator controls were received, False if timeout.
        """
        msg = self.mav.recv_match(
            type="HIL_ACTUATOR_CONTROLS",
            blocking=True,
            timeout=timeout,
        )

        if msg is not None:
            self.actuator_controls = list(msg.controls)
            return True

        return False

    def send_hil_sensor(
        self,
        time_usec: int,
        xacc: float = 0.0,
        yacc: float = 0.0,
        zacc: float = -9.81,
        xgyro: float = 0.0,
        ygyro: float = 0.0,
        zgyro: float = 0.0,
        xmag: float = 0.2,
        ymag: float = 0.0,
        zmag: float = 0.4,
        abs_pressure: float = 1013.25,
        diff_pressure: float = 0.0,
        pressure_alt: float = 0.0,
        temperature: float = 25.0,
        fields_updated: int = 0x1FFF,  # all fields updated
        id: int = 0,
    ):
        """
        Send IMU sensor data (HIL_SENSOR) message to PX4.

        Args:
            time_usec: Timestamp [us]
            xacc, yacc, zacc: Accelerometer readings [m/s^2]
            xgyro, ygyro, zgyro: Gyroscope readings [rad/s]
            xmag, ymag, zmag: Magnetometer readings [gauss]
            abs_pressure: Absolute pressure [hPa]
            diff_pressure: Differential pressure [hPa]
            pressure_alt: Altitude from pressure [m]
            temperature: Temperature [degC]
            fields_updated: Bitmask of updated fields
            id: IMU sensor instance (zero indexed)
        """
        self.proto.hil_sensor_send(
            time_usec,
            xacc,
            yacc,
            zacc,
            xgyro,
            ygyro,
            zgyro,
            xmag,
            ymag,
            zmag,
            abs_pressure,
            diff_pressure,
            pressure_alt,
            temperature,
            fields_updated,
            id=id,
        )

    def send_hil_gps(
        self,
        time_usec: int,
        fix_type: int = 3,  # 3D fix
        lat: int = 473977418,  # 47.3977418 deg (Zurich)
        lon: int = 85455939,  # 8.5455939 deg (Zurich)
        alt: int = 488000,  # 488 m AMSL (Zurich)
        eph: int = 100,  # 1.0 GPS HDOP
        epv: int = 100,  # 1.0 GPS VDOP
        vel: int = 0,  # stationary
        vn: int = 0,  # stationary
        ve: int = 0,  # stationary
        vd: int = 0,  # stationary
        cog: int = 65535,  # not available (UINT16_MAX)
        satellites_visible: int = 10,
        id: int = 0,
        yaw: int = 0,  # not available
    ):
        """
        Send GPS sensor data (HIL_GPS) message to PX4.

        Args:
            time_usec: Timestamp [us]
            fix_type: GPS fix type (0: no fix, 2: 2D fix, 3: 3D fix)
            lat: Latitude [deg] * 10^7
            lon: Longitude [deg] * 10^7
            alt: Altitude MSL [mm]
            eph: GPS HDOP [cm] * 10^2
            epv: GPS VDOP [cm] * 10^2
            vel: GPS ground speed [cm/s]
            vn, ve, vd: GPS NED velocity [cm/s]
            cog: Course over ground (not heading, but direction of movement) [cdeg]
            satellites_visible: Number of satellites visible
            id: GPS sensor instance (zero indexed),
            yaw: vehicle yaw relative to Earth's North [cdeg]

        """
        self.proto.hil_gps_send(
            time_usec,
            fix_type,
            lat,
            lon,
            alt,
            eph,
            epv,
            vel,
            vn,
            ve,
            vd,
            cog,
            satellites_visible,
            id=id,
            yaw=yaw,
        )

    def send_hil_state_quaternion(
        self,
        time_usec: int,
        attitude_quaternion: list = [1.0, 0.0, 0.0, 0.0],
        rollspeed: float = 0.0,
        pitchspeed: float = 0.0,
        yawspeed: float = 0.0,
        lat: int = 473977418,
        lon: int = 85455939,
        alt: int = 488000,
        vx: int = 0,
        vy: int = 0,
        vz: int = 0,
        ind_airspeed: int = 0,
        true_airspeed: int = 0,
        xacc: int = 0,
        yacc: int = 0,
        zacc: int = 0,
    ):
        """
        Send HIL_STATE_QUATERNION message to PX4.

        Args:
            time_usec: Timestamp in microseconds
            attitude_quaternion: [w, x, y, z] quaternion
            rollspeed, pitchspeed, yawspeed: Angular velocities [rad/s]
            lat, lon: Position [degE7]
            alt: Altitude MSL [mm]
            vx, vy, vz: Velocity NED [cm/s]
            ind_airspeed, true_airspeed: Airspeed [cm/s]
            xacc, yacc, zacc: Acceleration [mG]
        """

        self.proto.hil_state_quaternion_send(
            time_usec,
            attitude_quaternion,
            rollspeed,
            pitchspeed,
            yawspeed,
            lat,
            lon,
            alt,
            vx,
            vy,
            vz,
            ind_airspeed,
            true_airspeed,
            xacc,
            yacc,
            zacc,
        )

    def wait_for_px4(self, current_state: newton.State, body_qd_prev: np.ndarray):
        """Send sensor data until PX4 starts responding with actuator controls."""
        logger.info("Waiting for PX4 to start lockstep...")
        sim_time = 0
        while True:
            sim_time += self.sim_dt
            self._send_sensor_data(current_state, body_qd_prev, sim_time)
            if self.receive_actuator_controls(timeout=1.0):
                logger.info("PX4 lockstep established")
                return sim_time

    def _send_sensor_data(
        self, current_state: newton.State, body_qd_prev: np.ndarray, sim_time: float
    ):
        """Send simulated sensor data to PX4."""
        time_usec = int(sim_time * 1e6)

        # Get current state from simulation
        # body_q contains position and quaternion for each body
        # body_qd contains linear and angular velocity for each body
        body_q = current_state.body_q.numpy()
        body_qd = current_state.body_qd.numpy()
        body_qd_prev = body_qd_prev

        # Extract vehicle state (body index 0)
        # Position: [x, y, z]
        pos = body_q[0, :3]
        # Quaternion: [x, y, z, w] (warp convention)
        quat_xyzw = body_q[0, 3:7]
        # Convert to [w, x, y, z] for MAVLink
        quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]

        # Velocity: [vx, vy, vz, wx, wy, wz] (linear, angular)
        vel_linear = body_qd[0, :3]
        vel_linear_prev = body_qd_prev[0, :3]
        vel_angular = body_qd[0, 3:6]

        # Acceleration: [x, y, z]
        acc_linear = (vel_linear - vel_linear_prev) / self.sim_dt

        # IMU data (accelerometer measures specific force in body frame)
        # Specific force = linear_acceleration - gravity (both in body frame)
        # Transform world frame vectors to body frame using inverse quaternion rotation
        quat = wp.quat(
            float(quat_xyzw[0]),
            float(quat_xyzw[1]),
            float(quat_xyzw[2]),
            float(quat_xyzw[3]),
        )

        # Gravity in Newton's (FLU, i.e., Z-up) world frame (pointing down)
        gravity_world = wp.vec3(0.0, 0.0, -9.81)
        # Transform gravity to body frame
        gravity_body = wp.quat_rotate_inv(quat, gravity_world)

        # Linear acceleration in world frame, transform to body frame
        acc_world = wp.vec3(
            float(acc_linear[0]), float(acc_linear[1]), float(acc_linear[2])
        )
        acc_body = wp.quat_rotate_inv(quat, acc_world)

        # Accelerometer = specific force = acceleration - gravity (FRD body frame)
        xacc = acc_body[0] - gravity_body[0] + self.rng.gauss(0, 0.02)
        yacc = acc_body[1] - gravity_body[1] + self.rng.gauss(0, 0.02)
        zacc = acc_body[2] - gravity_body[2] + self.rng.gauss(0, 0.02)

        # Gyroscope (angular velocity in FRD body frame)
        vel_angular_body = wp.quat_rotate_inv(quat, wp.vec3(vel_angular))
        xgyro = vel_angular_body[0] + self.rng.gauss(0, 0.02)
        ygyro = vel_angular_body[1] + self.rng.gauss(0, 0.02)
        zgyro = vel_angular_body[2] + self.rng.gauss(0, 0.02)

        # Magnetometer - World Magnetic Model for Zurich (lat: 47.4°, lon: 8.5°)
        # Hardcoded WMM values for Zurich
        declination_rad = math.radians(3.0)  # ~3° East (magnetic north vs true north)
        inclination_rad = math.radians(64.0)  # ~64° dip angle (field points into earth)
        field_strength_gauss = 0.48

        # Construct magnetic field in NED frame
        # mag_ned = Dcm(Euler(0, -inclination, declination)) * [H, 0, 0]
        mag_n = (
            field_strength_gauss * math.cos(declination_rad) * math.cos(inclination_rad)
        )
        mag_e = field_strength_gauss * math.sin(declination_rad)
        mag_d = (
            field_strength_gauss * math.cos(declination_rad) * math.sin(inclination_rad)
        )

        # Convert NED to Newton world frame (X=North, Y=East, Z=Up)
        mag_world = wp.vec3(mag_n, mag_e, -mag_d)

        # Rotate to FRD body frame
        mag_body = wp.quat_rotate_inv(quat, mag_world)
        xmag = mag_body[0] + self.rng.gauss(0, 0.02)
        ymag = mag_body[1] + self.rng.gauss(0, 0.02)
        zmag = mag_body[2] + self.rng.gauss(0, 0.03)

        # Barometer
        # Approximate pressure from altitude (simplified model)
        altitude = pos[2]  # z is up in simulation
        sea_level_pressure = 1013.25  # hPa
        # Barometric formula approximation
        abs_pressure = sea_level_pressure * (
            1 - 2.25577e-5 * altitude
        ) ** 5.25588 + self.rng.gauss(0, 0.02)
        pressure_alt = altitude + self.rng.gauss(0, 0.02)

        # Send HIL_SENSOR at simulation rate
        self.send_hil_sensor(
            time_usec=time_usec,
            xacc=xacc,
            yacc=yacc,
            zacc=zacc,
            xgyro=xgyro,
            ygyro=ygyro,
            zgyro=zgyro,
            xmag=xmag,
            ymag=ymag,
            zmag=zmag,
            abs_pressure=abs_pressure,
            pressure_alt=pressure_alt,
        )

        # Send GPS at lower rate (10 Hz)
        if sim_time - self.last_hil_gps_time >= self.hil_gps_interval:
            self.last_hil_gps_time = sim_time

            # Convert simulation position to GPS coordinates
            # Using a reference point (Zurich) and adding local offsets
            # 1 degree latitude ~= 111km, 1 degree longitude ~= 111km * cos(lat)
            ref_lat = 47.3977418  # Reference latitude
            ref_lon = 8.5455939  # Reference longitude
            ref_alt = 488.0  # Reference altitude MSL [m]

            # Position offset in meters (x=East, y=North in typical GPS convention)
            # Simulation uses x=forward, y=left, z=up
            lat_offset = pos[0] / 111000.0  # Approximate
            lon_offset = pos[1] / (111000.0 * math.cos(math.radians(ref_lat)))

            lat = int((ref_lat + lat_offset) * 1e7)
            lon = int((ref_lon + lon_offset) * 1e7)
            alt = int((ref_alt + altitude) * 1000)  # mm

            # Velocity in cm/s: Newton world (X=N, Y=W, Z=Up) → NED
            vn = int(vel_linear[0] * 100)
            ve = int(-vel_linear[1] * 100)  # East = -West
            vd = int(-vel_linear[2] * 100)  # Down = -Up

            vel = int(math.sqrt(vel_linear[0] ** 2 + vel_linear[1] ** 2) * 100)

            self.send_hil_gps(
                time_usec=time_usec,
                lat=lat,
                lon=lon,
                alt=alt,
                vn=vn,
                ve=ve,
                vd=vd,
                vel=vel,
            )

            # Also send full state for visualization/logging
            self.send_hil_state_quaternion(
                time_usec=time_usec,
                attitude_quaternion=quat_wxyz,
                rollspeed=xgyro,
                pitchspeed=ygyro,
                yawspeed=zgyro,
                lat=lat,
                lon=lon,
                alt=alt,
                vx=vn,
                vy=ve,
                vz=vd,
                xacc=int(xacc * 1000 / 9.81),  # Convert to mG
                yacc=int(yacc * 1000 / 9.81),
                zacc=int(zacc * 1000 / 9.81),
            )
