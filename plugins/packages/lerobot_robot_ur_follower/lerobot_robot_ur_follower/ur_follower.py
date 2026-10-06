from typing import Any

from lerobot.robots import Robot

from .config_ur_follower import UrFollowerConfig

_PENDING = "Pendiente de implementar"


class UrFollower(Robot):
    """Adaptador entre LeRobot y `ur_core.UrFollowerCore`, pendiente de implementar.

    Hoy solo existe para que LeRobot encuentre `--robot.type=ur_follower`; cada método lanza
    NotImplementedError.
    """

    config_class = UrFollowerConfig
    name = "ur_follower"

    @property
    def observation_features(self) -> dict[str, Any]:
        raise NotImplementedError(_PENDING)

    @property
    def action_features(self) -> dict[str, Any]:
        raise NotImplementedError(_PENDING)

    @property
    def is_connected(self) -> bool:
        raise NotImplementedError(_PENDING)

    def connect(self, calibrate: bool = True) -> None:
        raise NotImplementedError(_PENDING)

    @property
    def is_calibrated(self) -> bool:
        raise NotImplementedError(_PENDING)

    def calibrate(self) -> None:
        raise NotImplementedError(_PENDING)

    def configure(self) -> None:
        raise NotImplementedError(_PENDING)

    def get_observation(self) -> dict[str, Any]:
        raise NotImplementedError(_PENDING)

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        raise NotImplementedError(_PENDING)

    def disconnect(self) -> None:
        raise NotImplementedError(_PENDING)
