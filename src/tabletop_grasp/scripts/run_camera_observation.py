#!/usr/bin/env python3
"""Bounded real-camera observation in domain 192; launches no motion nodes."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=45.)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--label', default='stationary_unmeasured')
    parser.add_argument('--rviz', action='store_true')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '192':
        parser.error('ROS_DOMAIN_ID=192 is required for this isolated camera evidence runner')
    if not 5 <= args.seconds <= 600:
        parser.error('duration must be within 5..600 seconds')
    args.output.mkdir(parents=True, exist_ok=True)
    launch_command = ['ros2', 'launch', 'tabletop_grasp', 'camera_observation.launch.py',
                      'camera_serial:=1', 'rviz:=' + str(args.rviz).lower()]
    if args.config is not None:
        launch_command.append('config:=' + str(args.config.resolve()))
    observer_command = [sys.executable, str(Path(__file__).with_name('capture_camera_evidence.py')),
                        '--output', str(args.output), '--seconds', str(args.seconds),
                        '--label', args.label]
    with (args.output / 'launch.log').open('w') as launch_log, \
            (args.output / 'observer.log').open('w') as observer_log:
        observer = subprocess.Popen(observer_command, stdout=observer_log,
                                    stderr=subprocess.STDOUT, start_new_session=True)
        launch = subprocess.Popen(launch_command, stdout=launch_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        try:
            result = observer.wait(timeout=args.seconds + 30.)
        finally:
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
                try:
                    launch.wait(timeout=20.)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGTERM)
                    launch.wait(timeout=10.)
            if observer.poll() is None:
                observer.terminate()
                observer.wait(timeout=5.)
    print('observer exit:', result, 'launch exit:', launch.returncode)
    print('Exit 0 means capture completed, not real-scene acceptance; inspect observation.json.')
    return result


if __name__ == '__main__':
    raise SystemExit(main())
