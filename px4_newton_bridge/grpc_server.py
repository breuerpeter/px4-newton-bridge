"""gRPC server exposing the NewtonAPI service."""

import math
from concurrent import futures

import grpc
import warp as wp

from newton_api.generated import newton_api_pb2, newton_api_pb2_grpc

from .logging import logger


def _shortest_arc_omega(current: wp.quat, target: wp.quat, duration: float) -> wp.vec3:
    """Compute constant angular velocity for shortest-path rotation.

    Python equivalent of newton.spatial.quat_velocity (which is @wp.func,
    only callable from Warp kernels).
    """
    delta = wp.normalize(wp.mul(target, wp.quat_inverse(current)))

    # Ensure shortest path (quaternion double-cover)
    if float(wp.dot(delta, wp.quat_identity())) < 0.0:
        delta = wp.quat(-delta[0], -delta[1], -delta[2], -delta[3])

    axis, angle = wp.quat_to_axis_angle(delta)
    return axis * (float(angle) / duration)


class _NewtonAPIServicer(newton_api_pb2_grpc.NewtonAPIServicer):
    """Implements the NewtonAPI gRPC interface."""

    def __init__(self, server: "GRPCServer"):
        self._server = server

    def SetPose(self, request, context):
        if len(request.pos) > 0:
            if len(request.pos) != 3:
                context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
                context.set_details("pos must have exactly 3 elements [x, y, z]")
                return newton_api_pb2.SetPoseResponse()
            x, y, z = request.pos
            cur = self._server.pos
            self._server.pos = wp.vec3(
                x if not math.isnan(x) else cur[0],
                y if not math.isnan(y) else cur[1],
                z if not math.isnan(z) else cur[2],
            )

        if len(request.quat_xyzw) > 0:
            if len(request.quat_xyzw) != 4:
                context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
                context.set_details("quat_xyzw must have exactly 4 elements [x, y, z, w]")
                return newton_api_pb2.SetPoseResponse()
            if not any(math.isnan(v) for v in request.quat_xyzw):
                x, y, z, w = request.quat_xyzw
                self._server.quat_xyzw = wp.quat(x, y, z, w)

        return newton_api_pb2.SetPoseResponse()

    def SetOmega(self, request, context):
        if len(request.omega) != 3:
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details("omega must have exactly 3 elements [x, y, z]")
            return newton_api_pb2.SetOmegaResponse()
        x, y, z = request.omega
        self._server.omega = wp.vec3(x, y, z)
        return newton_api_pb2.SetOmegaResponse()

    def SetGpsFixType(self, request, context):
        self._server.gps_fix_type = request.fix_type
        logger.info(f"GPS fix_type set to {self._server.gps_fix_type}")
        return newton_api_pb2.SetGpsFixTypeResponse()

    def ChangeAttTo(self, request, context):
        if len(request.quat_xyzw) != 4:
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details("quat_xyzw must have exactly 4 elements [x, y, z, w]")
            return newton_api_pb2.ChangeAttToResponse()

        if request.duration <= 0:
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details("duration must be positive")
            return newton_api_pb2.ChangeAttToResponse()

        x, y, z, w = request.quat_xyzw
        target = wp.quat(x, y, z, w)
        current = self._server.quat_xyzw

        omega = _shortest_arc_omega(current, target, request.duration)
        self._server.omega = omega
        self._server.att_transition = {
            "target": target,
            "remaining_time": request.duration,
        }
        logger.info(f"ChangeAttTo duration={request.duration:.2f}s")
        return newton_api_pb2.ChangeAttToResponse()


class GRPCServer:
    """gRPC server for external control of the bridge.

    Attributes:
        pos: position as wp.vec3
        quat_xyzw: quaternion as wp.quat [x, y, z, w]
        gps_fix_type: GPS fix type (0 = no fix, 3 = 3D fix)
    """

    def __init__(self, port: int = 4561):
        self.pos = wp.vec3(0.0, 0.0, 0.0)
        self.quat_xyzw = wp.quat_from_axis_angle(wp.vec3(1, 0, 0), wp.pi)
        self.omega = wp.vec3(0.0, 0.0, 0.0)
        self.gps_fix_type = 3  # 3D fix; set to 0 for no-fix
        self.att_transition = None  # active ChangeAttTo rotation

        self._grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
        newton_api_pb2_grpc.add_NewtonAPIServicer_to_server(_NewtonAPIServicer(self), self._grpc_server)
        self._grpc_server.add_insecure_port(f"0.0.0.0:{port}")
        self._grpc_server.start()
        logger.info(f"Listening on port {port}")

    def close(self):
        self._grpc_server.stop(grace=0)
