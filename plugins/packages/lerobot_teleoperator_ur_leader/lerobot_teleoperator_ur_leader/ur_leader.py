from typing import Any

from lerobot.teleoperators import Teleoperator

from .config_ur_leader import UrLeaderConfig

_PENDING = "Pendiente de implementar"


class UrLeader(Teleoperator):
    """Adaptador entre LeRobot y `ur_core.UrLeaderCore`, pendiente de implementar.

    Hoy solo existe para que LeRobot encuentre `--teleop.type=ur_leader`; cada método lanza
    NotImplementedError.
    """

    config_class = UrLeaderConfig
    name = "ur_leader"

    @property
    def action_features(self) -> dict[str, Any]:
        raise NotImplementedError(_PENDING)

    @property
    def feedback_features(self) -> dict[str, Any]:
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

    def get_action(self) -> dict[str, Any]:
        raise NotImplementedError(_PENDING)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError(_PENDING)

    def disconnect(self) -> None:
        raise NotImplementedError(_PENDING)
