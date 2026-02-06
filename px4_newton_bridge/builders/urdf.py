import glob

import newton
import warp as wp

from .builder_base import BuilderBase


class URDFBuilder(BuilderBase):
    """Build a vehicle model from a URDF file.

    Expects the URDF and its meshes to be in a ``urdf/`` subdirectory
    of the vehicle folder.
    """

    def build(self, builder: newton.ModelBuilder, body: int) -> None:
        urdf_dir = self.vehicle_dir / "urdf"
        urdf_files = glob.glob(str(urdf_dir / "*.urdf"))
        if len(urdf_files) != 1:
            raise FileNotFoundError(
                f"Expected exactly one URDF file in {urdf_dir}, found {len(urdf_files)}"
            )

        builder.add_urdf(
            urdf_files[0],
            xform=wp.transform_identity(),
            floating=True,
            collapse_fixed_joints=True,
            ignore_inertial_definitions=False,
            enable_self_collisions=False,
        )
