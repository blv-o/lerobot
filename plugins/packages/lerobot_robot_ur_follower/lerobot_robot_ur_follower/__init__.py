"""Plugin `ur_follower` de SPEC_003.

LeRobot importa este paquete al arrancar cualquier `lerobot-*` (register_third_party_plugins),
y eso registra `UrFollowerConfig` como `--robot.type=ur_follower`. `UrFollower` se reexporta aquí
porque make_device_from_device_class busca la clase sin el sufijo `Config` en este paquete.
"""

from .config_ur_follower import UrFollowerConfig
from .ur_follower import UrFollower

__all__ = ["UrFollower", "UrFollowerConfig"]
