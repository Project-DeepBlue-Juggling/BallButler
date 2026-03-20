"""Keyboard and gamepad input handling for the BallButler rover sim.

Maps user input to a velocity command (vx, vy, omega) that will later feed
into the swerve drive inverse kinematics.

Keyboard controls (via MuJoCo viewer key callback — tap to adjust):
    Up/Down arrows  — increase / decrease vx   (forward/back, 0.25 m/s steps)
    Left/Right      — increase / decrease vy   (strafe, 0.25 m/s steps)
    ,  / .          — rotate CCW / CW          (omega, 0.5 rad/s steps)
    Space           — zero all velocities (stop)

Gamepad (if connected, via pygame — analog, continuous):
    Left stick Y  — vx (forward/back)
    Left stick X  — vy (strafe)
    Right stick X — omega (rotation)
"""

from dataclasses import dataclass

import glfw

from config import SimConfig


@dataclass
class VelocityCommand:
    """Desired chassis velocity in the body frame."""
    vx: float = 0.0   # m/s, forward positive
    vy: float = 0.0   # m/s, left positive
    omega: float = 0.0  # rad/s, CCW positive


# Maximum velocity magnitudes
MAX_VX = 2.0      # m/s
MAX_VY = 2.0      # m/s
MAX_OMEGA = 3.0   # rad/s (~170 deg/s)

# Keyboard step sizes per tap
VX_STEP = 0.25    # m/s
VY_STEP = 0.25    # m/s
OMEGA_STEP = 0.5  # rad/s


class InputHandler:
    """Reads keyboard (via viewer callback) and optional gamepad, returns a VelocityCommand."""

    def __init__(self, cfg: SimConfig):
        self.cfg = cfg
        self.quit_requested = False

        # Keyboard-driven velocity (tap to adjust, persists between polls)
        self._kb_cmd = VelocityCommand()

        # Gamepad support (pygame joystick doesn't need window focus)
        self.joystick = None
        try:
            import pygame
            self._pygame = pygame
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                print(f"[InputHandler] Gamepad detected: {self.joystick.get_name()}")
            else:
                print("[InputHandler] No gamepad detected — using keyboard only.")
        except ImportError:
            self._pygame = None
            print("[InputHandler] pygame not available — gamepad disabled.")

    def key_callback(self, key: int):
        """Called by the MuJoCo viewer on key press. Adjusts velocity command."""
        cmd = self._kb_cmd

        if key == glfw.KEY_UP:
            cmd.vx = _clamp(cmd.vx + VX_STEP, MAX_VX)
        elif key == glfw.KEY_DOWN:
            cmd.vx = _clamp(cmd.vx - VX_STEP, MAX_VX)
        elif key == glfw.KEY_LEFT:
            cmd.vy = _clamp(cmd.vy + VY_STEP, MAX_VY)
        elif key == glfw.KEY_RIGHT:
            cmd.vy = _clamp(cmd.vy - VY_STEP, MAX_VY)
        elif key == glfw.KEY_COMMA:       # < key
            cmd.omega = _clamp(cmd.omega + OMEGA_STEP, MAX_OMEGA)
        elif key == glfw.KEY_PERIOD:      # > key
            cmd.omega = _clamp(cmd.omega - OMEGA_STEP, MAX_OMEGA)
        elif key == glfw.KEY_SPACE:
            cmd.vx = 0.0
            cmd.vy = 0.0
            cmd.omega = 0.0

    def poll(self) -> VelocityCommand:
        """Return the current velocity command (keyboard state + gamepad override)."""
        cmd = VelocityCommand(
            vx=self._kb_cmd.vx,
            vy=self._kb_cmd.vy,
            omega=self._kb_cmd.omega,
        )

        # Gamepad overrides keyboard if any stick is active
        if self.joystick is not None and self._pygame is not None:
            self._pygame.event.pump()  # Process joystick events
            dz = self.cfg.gamepad_deadzone

            raw_vx = -self.joystick.get_axis(1)     # Y axis inverted
            raw_vy = -self.joystick.get_axis(0)      # X axis inverted for left-positive
            raw_omega = -self.joystick.get_axis(3)    # Right stick X, inverted for CCW-positive

            gp_vx = raw_vx if abs(raw_vx) > dz else 0.0
            gp_vy = raw_vy if abs(raw_vy) > dz else 0.0
            gp_omega = raw_omega if abs(raw_omega) > dz else 0.0

            if any(v != 0.0 for v in (gp_vx, gp_vy, gp_omega)):
                cmd.vx = gp_vx * MAX_VX
                cmd.vy = gp_vy * MAX_VY
                cmd.omega = gp_omega * MAX_OMEGA

        return cmd

    def close(self):
        if self._pygame is not None:
            self._pygame.quit()


def _clamp(value: float, limit: float) -> float:
    """Clamp value to [-limit, +limit], snapping to zero if within half a step."""
    clamped = max(-limit, min(limit, value))
    # Snap to zero to avoid floating-point drift from repeated stepping
    if abs(clamped) < 0.01:
        return 0.0
    return round(clamped, 4)
