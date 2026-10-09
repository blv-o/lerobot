import logging
from collections.abc import Callable
from typing import Any

from ur_core import FOLLOWERS, JOINT_NAMES, confirm_and_check, load_config

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot

from .config_ur_follower import UrFollowerConfig

log = logging.getLogger(__name__)


class UrFollower(Robot):
    """Adaptador entre LeRobot y `ur_core.UrFollowerCore`: solo traduce; la lógica vive en el núcleo.

    `ask` (por defecto `input`) responde la confirmación de `connect()`; los tests la sustituyen.
    """

    config_class = UrFollowerConfig
    name = "ur_follower"

    def __init__(self, config: UrFollowerConfig) -> None:
        super().__init__(config)
        self.config = config
        # El YAML se lee aquí y no en connect(): un error de configuración sale antes de conectar nada.
        teleop_config = load_config(config.config_path)
        self.core = FOLLOWERS[teleop_config.follower.type](teleop_config)
        self.cameras = make_cameras_from_configs(config.cameras)
        self.ask: Callable[[str], str] = input

    @property
    def observation_features(self) -> dict[str, Any]:
        cameras = {key: (cam.height, cam.width, 3) for key, cam in self.cameras.items()}
        return {**self.action_features, **cameras}

    @property
    def action_features(self) -> dict[str, Any]:
        return {f"{joint}.pos": float for joint in JOINT_NAMES}

    @property
    def is_connected(self) -> bool:
        # Sigue en True tras una parada del follower hasta disconnect(): LeRobot lo ve por la
        # excepción de get_observation()/send_action(), y así su `finally` siempre lo cierra.
        return self.core.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if not confirm_and_check(self.core.start_prompt(), self.core.check_start, ask=self.ask):
            log.warning("arranque cancelado por el operador")
            raise KeyboardInterrupt  # como Ctrl+C: LeRobot termina sin entrar en su bucle
        self.core.connect()
        try:
            for cam in self.cameras.values():
                cam.connect()
        except BaseException:
            self.core.disconnect()  # armado y sin cámaras: LeRobot no llamará a disconnect()
            raise

    @property
    def is_calibrated(self) -> bool:
        return True  # sin pinza no hay nada que calibrar: el UR ya da sus ángulos en rad

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_observation(self) -> dict[str, Any]:
        """Pose medida y cámaras; tras una parada del follower lanza `FollowerStoppedError`."""
        q_rad = self.core.get_joints()
        observation: dict[str, Any] = {f"{joint}.pos": q for joint, q in zip(JOINT_NAMES, q_rad, strict=True)}
        for key, cam in self.cameras.items():
            observation[key] = cam.read_latest()
        return observation

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        q_rad = [float(action[f"{joint}.pos"]) for joint in JOINT_NAMES]
        self.core.send_joints(q_rad)
        return {f"{joint}.pos": q for joint, q in zip(JOINT_NAMES, q_rad, strict=True)}

    def disconnect(self) -> None:
        """Idempotente y sin lanzar: en el `finally` de `lerobot-record` va antes de cerrar el leader."""
        try:
            for key, cam in self.cameras.items():
                if cam.is_connected:
                    self._disconnect_camera(key, cam)
        finally:
            self.core.disconnect()

    @staticmethod
    def _disconnect_camera(key: str, cam: Any) -> None:
        try:
            cam.disconnect()
        except Exception:
            # Se registra y se sigue: una cámara que no cierra no debe dejar el follower armado.
            log.exception("ur_follower: no se pudo cerrar la cámara %s", key)
