"""Shared constants for joint, actuator, and body names.

Module order throughout the codebase: FL, FR, RL, RR.
"""

PREFIXES = ["fl", "fr", "rl", "rr"]

YAW_JOINT_NAMES = [f"{p}_yaw" for p in PREFIXES]
DRIVE_JOINT_NAMES = [f"{p}_drive" for p in PREFIXES]
SUSP_JOINT_NAMES = [f"{p}_susp" for p in PREFIXES]

YAW_ACT_NAMES = [f"{p}_yaw_act" for p in PREFIXES]
DRIVE_ACT_NAMES = [f"{p}_drive_act" for p in PREFIXES]

WHEEL_BODY_NAMES = [f"{p}_wheel" for p in PREFIXES]
