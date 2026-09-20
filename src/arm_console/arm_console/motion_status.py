"""Operator-facing status for commands sent by Arm Console."""

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionStatus:
    """Describe the last explicit command without inferring completion."""

    state: str
    detail: str
