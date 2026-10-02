"""Teleoperación leader/follower de robots UR por RTDE (SPEC_003).

No importa LeRobot: los plugins `lerobot_robot_ur_follower` y `lerobot_teleoperator_ur_leader`
son la única capa que habla con él (principio 1 de project/CONSTITUTION.md).
"""

from ur_core.config import ConfigError, TeleopConfig, load_config

__all__ = ["ConfigError", "TeleopConfig", "load_config"]
