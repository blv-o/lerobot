from dataclasses import dataclass
from pathlib import Path

from lerobot.teleoperators import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("ur_leader")
@dataclass
class UrLeaderConfig(TeleoperatorConfig):
    """Solo la ruta del YAML compartido.

    Todo lo propio del leader (tipo, IP, timeout) vive en el YAML compartido con `ur_follower`.
    """

    config_path: Path
