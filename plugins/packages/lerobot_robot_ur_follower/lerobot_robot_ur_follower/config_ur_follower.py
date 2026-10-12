from dataclasses import dataclass, field
from pathlib import Path

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("ur_follower")
@dataclass
class UrFollowerConfig(RobotConfig):
    """Solo la ruta del YAML compartido y las cámaras.

    Lo propio del UR (IP, posición inicial) vive en el YAML compartido con `ur_leader`, y los
    ajustes del movimiento en el programa del TP; las cámaras llegan por la CLI estándar de LeRobot.
    """

    config_path: Path
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
