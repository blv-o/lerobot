from dataclasses import dataclass, field
from pathlib import Path

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("ur_follower")
@dataclass
class UrFollowerConfig(RobotConfig):
    """Todo lo propio del UR (IP, servo, watchdog, posición inicial) vive en el YAML compartido
    con `ur_leader`; aquí solo va su ruta y las cámaras, que llegan por la CLI estándar."""

    config_path: Path
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
