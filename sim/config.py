"""Simulation configuration parameters for the BallButler rover MuJoCo sim."""

from dataclasses import dataclass, field
from pathlib import Path

# Paths
SIM_DIR = Path(__file__).parent
MODELS_DIR = SIM_DIR / "models"
LOGS_DIR = SIM_DIR / "logs"

SCENE_XML = MODELS_DIR / "scene.xml"
TERRAIN_SCENE_XML = MODELS_DIR / "scene_terrain.xml"


@dataclass
class SimConfig:
    """Top-level simulation configuration."""

    # Physics
    timestep: float = 0.002  # 2 ms (500 Hz physics)
    control_rate_hz: float = 50.0  # 50 Hz control loop

    # Rendering
    window_width: int = 1280
    window_height: int = 720
    render_fps: float = 60.0  # Target render framerate

    # Input
    gamepad_deadzone: float = 0.1

    # Logging
    log_enabled: bool = True
    log_flush_interval_s: float = 5.0  # Flush CSV every N seconds

    @property
    def control_decimation(self) -> int:
        """Number of physics steps per control step."""
        return int(round(1.0 / (self.timestep * self.control_rate_hz)))

    @property
    def render_decimation(self) -> int:
        """Number of physics steps per render frame."""
        return max(1, int(round(1.0 / (self.timestep * self.render_fps))))
