"""Merge independently published model joint states without inventing values."""

from copy import deepcopy

import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import JointState

from .moveit_adapter import ARM_JOINTS, PASSIVE_JOINT


class JointFeedback:
    def __init__(self, timeout_sec, future_tolerance_sec):
        self.timeout_ns = int(timeout_sec * 1e9)
        self.future_ns = int(future_tolerance_sec * 1e9)
        self.values = {}

    def update(self, message, now_ns):
        names = ARM_JOINTS + (PASSIVE_JOINT,)
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        if (len(message.name) != len(message.position) or len(set(message.name)) != len(message.name)
                or not message.name or not np.isfinite(message.position).all()
                or stamp <= 0 or not -self.future_ns <= now_ns - stamp <= self.timeout_ns):
            self.values.clear()
            raise ValueError('invalid or stale model joint feedback')
        for name, position in zip(message.name, message.position):
            if name in names and stamp > self.values.get(name, (0, 0))[1]:
                self.values[name] = (float(position), stamp)
        if any(name not in self.values for name in names):
            return None
        oldest = min(self.values[name][1] for name in names)
        if now_ns - oldest > self.timeout_ns:
            return None
        result = JointState(header=deepcopy(message.header), name=list(names),
                            position=[self.values[name][0] for name in names])
        result.header.stamp = Time(nanoseconds=oldest).to_msg()
        return result
