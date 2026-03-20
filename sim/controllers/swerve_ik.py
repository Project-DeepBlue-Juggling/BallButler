"""Swerve drive inverse kinematics for a 4-wheel rover.

Given a desired chassis velocity (vx, vy, omega) in the body frame, computes
the required wheel speed (rad/s) and yaw angle (rad) for each of the 4 modules.

Includes the 180-degree flip optimisation: if the target yaw angle is more than
90 degrees from the current angle, reverse the wheel direction and use the
supplementary angle.  This avoids slow 180-degree yaw rotations.

Coordinate frame: x-forward, y-left, z-up (matches the MJCF model).

Module layout (top view):
    FL (+x,+y) ---- FR (+x,-y)
         |     chassis    |
    RL (-x,+y) ---- RR (-x,-y)
"""

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass
class ModuleCommand:
    """Commanded state for a single swerve module."""
    yaw_angle: float = 0.0    # rad, target yaw angle
    wheel_speed: float = 0.0  # rad/s, target wheel angular velocity


# Module positions relative to chassis centre [x, y] in metres
# Order: FL, FR, RL, RR (matches actuator order in the MJCF)
MODULE_POSITIONS = [
    (+0.225, +0.225),  # front-left
    (+0.225, -0.225),  # front-right
    (-0.225, +0.225),  # rear-left
    (-0.225, -0.225),  # rear-right
]

WHEEL_RADIUS = 0.0825  # metres

# Below this chassis speed, don't rotate yaw modules (prevents jitter at rest)
SPEED_DEADBAND = 0.01  # m/s


def swerve_ik(
    vx: float,
    vy: float,
    omega: float,
    current_yaw_angles: List[float],
) -> List[ModuleCommand]:
    """Compute swerve module commands from a chassis velocity command.

    Args:
        vx: Forward velocity (m/s), positive = forward.
        vy: Lateral velocity (m/s), positive = left.
        omega: Rotational velocity (rad/s), positive = CCW.
        current_yaw_angles: Current yaw angle of each module [FL, FR, RL, RR] (rad).

    Returns:
        List of 4 ModuleCommand (FL, FR, RL, RR).
    """
    commands = []
    # Track the fastest wheel — used in Phase 4 for wheel speed normalisation
    # (if any wheel exceeds the motor's speed limit, all wheels scale down proportionally)
    max_wheel_speed = 0.0

    for i, (mx, my) in enumerate(MODULE_POSITIONS):
        # Velocity contribution from rotation at this module position
        # v_rot = omega × r  (where r is the position vector from CoR to module)
        # In 2D:  v_rot_x = -omega * my,  v_rot_y = omega * mx
        wx = vx + (-omega * my)
        wy = vy + (omega * mx)

        # Wheel ground speed (m/s) and direction
        wheel_ground_speed = math.hypot(wx, wy)

        if wheel_ground_speed < SPEED_DEADBAND:
            # Near-zero speed: hold current yaw angle, zero wheel speed
            commands.append(ModuleCommand(
                yaw_angle=current_yaw_angles[i],
                wheel_speed=0.0,
            ))
            continue

        # Target yaw angle (atan2 gives angle of the velocity vector)
        target_angle = math.atan2(wy, wx)

        # 180-degree flip optimisation
        target_angle, wheel_ground_speed = _optimize_angle(
            target_angle, current_yaw_angles[i], wheel_ground_speed
        )

        # Convert ground speed to wheel angular velocity
        wheel_angular_speed = wheel_ground_speed / WHEEL_RADIUS

        commands.append(ModuleCommand(
            yaw_angle=target_angle,
            wheel_speed=wheel_angular_speed,
        ))

        max_wheel_speed = max(max_wheel_speed, abs(wheel_angular_speed))

    return commands


def _optimize_angle(
    target: float, current: float, speed: float
) -> Tuple[float, float]:
    """Apply the 180-degree flip optimisation.

    If the target angle is more than 90 degrees from the current angle,
    flip the target by 180 degrees and reverse the wheel direction.
    This keeps the yaw motor within a ±90-degree rotation of its current
    position, which is faster and avoids unnecessary full rotations.
    """
    # Normalise the angular difference to [-pi, pi]
    delta = _wrap_angle(target - current)

    if abs(delta) > math.pi / 2:
        # Flip: reverse wheel and rotate by 180 less
        target = _wrap_angle(target + math.pi)
        speed = -speed

    return target, speed


def _wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi
