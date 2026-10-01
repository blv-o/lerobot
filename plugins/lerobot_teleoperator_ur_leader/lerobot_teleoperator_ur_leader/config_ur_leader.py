from dataclasses import dataclass
from pathlib import Path

from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("ur_leader")
@dataclass
class UrLeaderConfig(TeleoperatorConfig):
    """Todo lo propio del leader (tipo, IP, timeout) vive en el YAML compartido con
    `ur_follower`; aquí solo va su ruta."""

    config_path: Path
