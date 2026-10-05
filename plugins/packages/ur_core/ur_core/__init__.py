"""Teleoperación leader/follower de robots UR por RTDE.

No importa LeRobot: los plugins `lerobot_robot_ur_follower` y `lerobot_teleoperator_ur_leader`
son la única capa que habla con él, así un cambio interno de LeRobot no llega hasta aquí.
"""

from ur_core.config import JOINT_NAMES, ConfigError, TeleopConfig, load_config
from ur_core.follower import FOLLOWERS, FollowerStatus, FollowerStoppedError, UrFollowerCore
from ur_core.motion import interpolate, limit_step
from ur_core.startup import confirm_and_check, pose_errors
from ur_core.streaming import FollowerStartError, FollowerState

__all__ = [
    "FOLLOWERS",
    "JOINT_NAMES",
    "ConfigError",
    "FollowerStartError",
    "FollowerState",
    "FollowerStatus",
    "FollowerStoppedError",
    "TeleopConfig",
    "UrFollowerCore",
    "confirm_and_check",
    "interpolate",
    "limit_step",
    "load_config",
    "pose_errors",
]
