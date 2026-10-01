"""Plugin `ur_leader` de SPEC_003.

LeRobot importa este paquete al arrancar cualquier `lerobot-*` (register_third_party_plugins),
y eso registra `UrLeaderConfig` como `--teleop.type=ur_leader`. `UrLeader` se reexporta aquí
porque make_device_from_device_class busca la clase sin el sufijo `Config` en este paquete.
"""

from .config_ur_leader import UrLeaderConfig
from .ur_leader import UrLeader

__all__ = ["UrLeader", "UrLeaderConfig"]
