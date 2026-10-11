"""Teleoperación leader/follower de robots UR por RTDE.

No importa LeRobot: los plugins `lerobot_robot_ur_follower` y `lerobot_teleoperator_ur_leader`
son la única capa que habla con él, así un cambio interno de LeRobot no llega hasta aquí.
"""

from ur_core.config import JOINT_NAMES, ConfigError, TeleopConfig, load_config
from ur_core.follower import FollowerStoppedError, UrFollowerCore
from ur_core.leader import LeaderReadError, UrLeaderCore
from ur_core.startup import confirm_start
from ur_core.streaming import FollowerState

__all__ = [
    "JOINT_NAMES",
    "ConfigError",
    "FollowerState",
    "FollowerStoppedError",
    "LeaderReadError",
    "TeleopConfig",
    "UrFollowerCore",
    "UrLeaderCore",
    "confirm_start",
    "load_config",
]
