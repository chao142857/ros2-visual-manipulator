#!/usr/bin/env python3
"""Run only the isolated synthetic fixture; stop the launch process gracefully."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--seconds', type=float, default=45.)
parser.add_argument('--rviz', action='store_true')
args = parser.parse_args()
if os.environ.get('ROS_DOMAIN_ID') != '191':
    raise SystemExit('ROS_DOMAIN_ID=191 is required')
args.output.mkdir(parents=True, exist_ok=True)
observer_script = Path(__file__).with_name('observe_stack.py')
with (args.output / 'launch.log').open('w') as launch_log, \
        (args.output / 'observer.log').open('w') as observer_log:
    observer = subprocess.Popen([sys.executable, str(observer_script), '--mode', 'synthetic',
                                 '--output', str(args.output), '--seconds', str(args.seconds)],
                                stdout=observer_log, stderr=subprocess.STDOUT, start_new_session=True)
    launch = subprocess.Popen(['ros2', 'launch', 'tabletop_grasp', 'synthetic_demo.launch.py',
                               'rviz:=' + str(args.rviz).lower()],
                              stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        result = observer.wait(timeout=args.seconds + 15.)
    finally:
        # Signal the launch parent once. It owns orderly child shutdown.
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
raise SystemExit(result)
