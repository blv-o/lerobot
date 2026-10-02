"""Teleoperación leader/follower de robots UR por RTDE.

No importa LeRobot: los plugins `lerobot_robot_ur_follower` y `lerobot_teleoperator_ur_leader`
son la única capa que habla con él, así un cambio interno de LeRobot no llega hasta aquí.
"""

from ur_core.config import ConfigError, TeleopConfig, load_config

__all__ = ["ConfigError", "TeleopConfig", "load_config"]
