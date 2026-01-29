import os
import time
from typing import cast

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

    def __init__(self, ip: str = "0.0.0.0", sysid: int = 1, compid: int = 200):
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

        self.last_hb_time = 0.0
        self.last_hil_sensor_time = 0.0
        self.last_hil_gps_time = 0.0

        self.hb_interval = 1.0 / 1.0  # [s]
        self.hil_sensor_interval = 1.0 / 250.0  # [s]
        self.hil_gps_interval = 1.0 / 10.0  # [s]

    def send_heartbeat(self):
        """
        Send hearbeat (HEARTBEAT) message to PX4.
        """
        now = time.time()
        if now - self.last_hb_time >= self.hb_interval:
            self.proto.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0,  # base_mode
                0,  # custom_mode
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self.last_hb_time = now

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
