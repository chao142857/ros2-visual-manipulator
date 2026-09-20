"""Convert an optical-frame hand-eye solution into a camera-link mount."""

import argparse
from pathlib import Path

from .frame_contract import tool_to_mount_from_optical
from .hand_eye_calibration import (
    Calibration, load_calibration, save_calibration,
)


def convert(optical_calibration_file, mount_optical_calibration_file,
            output_file, mount_frame='wrist_d405_link', overwrite=False):
    optical = load_calibration(optical_calibration_file)
    internal = load_calibration(mount_optical_calibration_file)
    if internal.tool_frame != mount_frame:
        raise ValueError(
            f'internal TF parent is {internal.tool_frame!r}, expected {mount_frame!r}'
        )
    if internal.camera_frame != optical.camera_frame:
        raise ValueError('internal TF optical child does not match solved optical frame')
    result = Calibration(
        tool_to_mount_from_optical(
            optical.T_tool_camera, internal.T_tool_camera
        ),
        optical.tool_frame, mount_frame,
        optical.method + '+realsense_internal_tf', optical.metrics,
    )
    return save_calibration(output_file, result, overwrite=overwrite)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Convert T_tool_optical plus T_mount_optical to T_tool_mount'
    )
    parser.add_argument('--optical-calibration', required=True)
    parser.add_argument('--mount-optical-calibration', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--mount-frame', default='wrist_d405_link')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args(argv)
    saved = convert(
        args.optical_calibration, args.mount_optical_calibration,
        args.output, args.mount_frame, args.overwrite,
    )
    print(f'MOUNT_CALIBRATION_SAVED {Path(saved)}')

