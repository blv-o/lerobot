from typing import Any

from lerobot.teleoperators import Teleoperator

from .config_ur_leader import UrLeaderConfig

_PENDING = "Pendiente de la fase F3 de SPEC_003"


class UrLeader(Teleoperator):
    """Adaptador entre LeRobot y `ur_teleop_core.URLeader` (se completa en F3)."""

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
