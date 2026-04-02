"""gRPC client for the NewtonAPI service."""

import grpc

from .generated import newton_api_pb2, newton_api_pb2_grpc


class NewtonClient:
    """Client for the Newton bridge gRPC service.

    Args:
        target: gRPC target string, e.g. "127.0.0.1:4561"
    """

    def __init__(self, target: str = "127.0.0.1:4561"):
        self.channel = grpc.insecure_channel(target)
        self.stub = newton_api_pb2_grpc.NewtonAPIStub(self.channel)

    def set_pose(
        self,
        pos: list[float] | None = None,
        quat_xyzw: list[float] | None = None,
    ):
        """Set the vehicle pose. NaN values are ignored (current value is kept).

        Args:
            pos: Position as [x, y, z] in meters.
            quat_xyzw: Quaternion as [x, y, z, w].
        """
        req = newton_api_pb2.SetPoseRequest()
        if pos is not None:
            req.pos.extend(pos)
        if quat_xyzw is not None:
            req.quat_xyzw.extend(quat_xyzw)
        self.stub.SetPose(req)

    def set_omega(self, omega: list[float]):
        """Set the vehicle angular velocity in world frame [rad/s].

        Args:
            omega: Angular velocity as [x, y, z] in rad/s.
        """
        req = newton_api_pb2.SetOmegaRequest()
        req.omega.extend(omega)
        self.stub.SetOmega(req)

    def set_gps_fix_type(self, fix_type: int):
        """Set the GPS fix type (0 = no fix, 3 = 3D fix)."""
        self.stub.SetGpsFixType(newton_api_pb2.SetGpsFixTypeRequest(fix_type=fix_type))

    def change_att_to(self, quat_xyzw: list[float], duration: float):
        """Smoothly rotate to target attitude over duration seconds.

        The bridge computes the shortest-path rotation and applies a constant
        angular velocity. Returns immediately; the rotation completes after
        ``duration`` seconds of sim time.

        Args:
            quat_xyzw: Target quaternion as [x, y, z, w].
            duration: Rotation time in seconds.
        """
        self.stub.ChangeAttTo(newton_api_pb2.ChangeAttToRequest(quat_xyzw=quat_xyzw, duration=duration))

    def close(self):
        self.channel.close()
