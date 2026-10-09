import logging
from collections.abc import Callable
from typing import Any

from ur_core import JOINT_NAMES, UrLeaderCore, confirm_and_check, load_config

from lerobot.teleoperators import Teleoperator

from .config_ur_leader import UrLeaderConfig

log = logging.getLogger(__name__)


class UrLeader(Teleoperator):
    """Adaptador entre LeRobot y `ur_core.UrLeaderCore`: solo traduce; la lógica vive en el núcleo.

    `ask` (por defecto `input`) responde la confirmación de `connect()`; los tests la sustituyen.
    """

    config_class = UrLeaderConfig
    name = "ur_leader"

    def __init__(self, config: UrLeaderConfig) -> None:
        super().__init__(config)
        self.config = config
        # El YAML se lee aquí y no en connect(): un error de configuración sale antes de conectar nada.
        self.core = UrLeaderCore(load_config(config.config_path))
        self.ask: Callable[[str], str] = input

    @property
    def action_features(self) -> dict[str, Any]:
        return {f"{joint}.pos": float for joint in JOINT_NAMES}

    @property
    def feedback_features(self) -> dict[str, Any]:
        return {}  # al leader no se le escribe nada

    @property
    def is_connected(self) -> bool:
        return self.core.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if not confirm_and_check(self.core.start_prompt(), self.core.check_start, ask=self.ask):
            log.warning("arranque cancelado por el operador")
            raise KeyboardInterrupt  # como Ctrl+C: LeRobot termina sin entrar en su bucle
        self.core.connect()

    @property
    def is_calibrated(self) -> bool:
        return True  # el UR ya da sus ángulos en rad: no hay nada que calibrar

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, Any]:
        """Pose del leader; si deja de dar datos lanza `LeaderReadError` y el bucle de LeRobot termina."""
        return {f"{joint}.pos": q for joint, q in zip(JOINT_NAMES, self.core.read_joints(), strict=True)}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass  # `feedback_features` está vacío: al leader no se le envía nada

    def disconnect(self) -> None:
        """Idempotente y sin lanzar: en `lerobot-teleoperate` va antes de cerrar el follower."""
        self.core.disconnect()
