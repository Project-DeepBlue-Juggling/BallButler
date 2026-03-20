"""Coordinate and orientation transforms."""

import math


def quat_to_euler(qw, qx, qy, qz):
    """Convert quaternion (wxyz) to (roll, pitch, heading) ZYX Euler angles.

    Returns:
        Tuple of (roll, pitch, heading) in radians.
        - roll:    rotation about the body x-axis
        - pitch:   rotation about the body y-axis
        - heading: rotation about the world z-axis
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation) — clamp for numerical safety
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Heading / yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    heading = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, heading
