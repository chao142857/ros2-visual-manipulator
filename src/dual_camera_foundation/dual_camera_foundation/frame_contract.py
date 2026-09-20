            
"""Small helpers documenting the D405 mount/optical frame contract."""

from .hand_eye_calibration import as_transform, inverse


def tool_to_mount_from_optical(T_tool_optical, T_mount_optical):
    """Return ``T_tool_mount`` from a solved optical transform.

    The RealSense driver publishes ``T_mount_optical`` internally. The
    project-owned static publisher must publish only the resulting
    ``tool0 -> wrist_d405_link`` transform.
    """
    return as_transform(T_tool_optical, 'T_tool_optical') @ inverse(
        as_transform(T_mount_optical, 'T_mount_optical')
    )

        

