"""Validation and persistence helpers for fixed camera extrinsics."""

import math
import os
from dataclasses import dataclass
from pathlib import Path
import tempfile

from geometry_msgs.msg import TransformStamped
import yaml


EXTRINSICS_NODE_NAME = 'camera_extrinsics_publisher_node'


@dataclass(frozen=True)
class CameraExtrinsics:
    """A validated base-to-camera rigid transform."""

    parent_frame: str
    child_frame: str
    xyz: tuple
    quaternion: tuple

    @classmethod
    def from_values(cls, parent_frame, child_frame, xyz, quaternion):
        """Validate values and return extrinsics with a unit quaternion."""
        parent_frame = str(parent_frame).strip()
        child_frame = str(child_frame).strip()

        if not parent_frame or not child_frame:
            raise ValueError('parent_frame and child_frame must be non-empty')
        if parent_frame == child_frame:
            raise ValueError('parent_frame and child_frame must be different')

        try:
            xyz = tuple(float(value) for value in xyz)
            quaternion = tuple(float(value) for value in quaternion)
        except (TypeError, ValueError) as error:
            raise ValueError(
                'xyz and quaternion must be numeric arrays'
            ) from error

        if len(xyz) != 3:
            raise ValueError('xyz must contain exactly 3 values')
        if len(quaternion) != 4:
            raise ValueError('quaternion must contain exactly 4 values')
        if not all(math.isfinite(value) for value in xyz + quaternion):
            raise ValueError(
                'xyz and quaternion must contain only finite values'
            )

        quaternion_norm = math.sqrt(sum(
            value * value for value in quaternion
        ))
        if quaternion_norm <= 1.0e-12:
            raise ValueError('quaternion norm must be greater than zero')

        normalized_quaternion = tuple(
            value / quaternion_norm for value in quaternion
        )
        return cls(
            parent_frame=parent_frame,
            child_frame=child_frame,
            xyz=xyz,
            quaternion=normalized_quaternion,
        )

    def to_transform_stamped(self, stamp):
        """Build a TransformStamped for a supplied ROS timestamp."""
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.parent_frame
        message.child_frame_id = self.child_frame
        message.transform.translation.x = self.xyz[0]
        message.transform.translation.y = self.xyz[1]
        message.transform.translation.z = self.xyz[2]
        message.transform.rotation.x = self.quaternion[0]
        message.transform.rotation.y = self.quaternion[1]
        message.transform.rotation.z = self.quaternion[2]
        message.transform.rotation.w = self.quaternion[3]
        return message

    def to_parameter_document(self):
        """Return a ROS 2 parameter-file document for the runtime node."""
        return {
            EXTRINSICS_NODE_NAME: {
                'ros__parameters': {
                    'calibrated': True,
                    'parent_frame': self.parent_frame,
                    'child_frame': self.child_frame,
                    'xyz': list(self.xyz),
                    'quaternion': list(self.quaternion),
                }
            }
        }


def save_camera_extrinsics(path, extrinsics, overwrite=False):
    """Atomically save extrinsics, refusing replacement unless requested."""
    if not str(path).strip():
        raise ValueError('extrinsics_file must be set when saving is enabled')
    if not isinstance(extrinsics, CameraExtrinsics):
        raise TypeError('extrinsics must be a CameraExtrinsics instance')

    requested_path = Path(str(path)).expanduser()
    requested_path.parent.mkdir(parents=True, exist_ok=True)

    # A symlink is common with colcon --symlink-install. Resolve it so saving
    # updates the source config instead of replacing the installed symlink.
    if requested_path.is_symlink():
        target_path = requested_path.resolve()
    else:
        target_path = requested_path.parent.resolve() / requested_path.name

    if target_path.exists() and not overwrite:
        raise FileExistsError(
            f'{target_path} already exists; explicit overwrite is required'
        )

    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=str(target_path.parent),
            prefix=f'.{target_path.name}.',
            suffix='.tmp',
            delete=False,
        ) as temporary_file:
            temporary_name = temporary_file.name
            yaml.safe_dump(
                extrinsics.to_parameter_document(),
                temporary_file,
                sort_keys=False,
            )
            temporary_file.flush()
            os.fsync(temporary_file.fileno())

        os.chmod(temporary_name, 0o644)
        if overwrite:
            os.replace(temporary_name, target_path)
        else:
            try:
                os.link(temporary_name, target_path)
            except FileExistsError as error:
                raise FileExistsError(
                    f'{target_path} appeared while saving; not overwritten'
                ) from error
            os.unlink(temporary_name)
        temporary_name = None
    finally:
        if temporary_name is not None:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass

    return str(target_path)
