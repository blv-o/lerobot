"""Adaptador `UrFollower(Robot)`: solo traduce entre LeRobot y `UrFollowerCore`.

Casi todos los tests sustituyen el núcleo por `FakeCore`, que registra el orden de las llamadas;
el último usa el núcleo real con el bucle del follower en un hilo, para que el doble no se aleje
de la API de verdad.
"""

import logging
import math
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from fakes import START_Q_RAD, FakeRTDE, packets
from lerobot_robot_ur_follower import UrFollower, UrFollowerConfig
from test_follower_core import Rig
from ur_core import JOINT_NAMES, ConfigError, FollowerStoppedError, UrFollowerCore, load_config

from lerobot.cameras.opencv import OpenCVCameraConfig

TEMPLATE = Path(__file__).parents[2] / "configs" / "ur_config.yaml"
JOINT_KEYS = [f"{joint}.pos" for joint in JOINT_NAMES]


class FakeCore:
    """`UrFollowerCore` de mentira: anota en `calls` (compartida con las cámaras) cada llamada."""

    def __init__(self, calls: list[str], checks: list[list[str]] | None = None) -> None:
        self.calls = calls
        self._checks = iter(checks or [[]])
        self.is_connected = False
        self.joints_rad = list(START_Q_RAD)
        self.get_error: Exception | None = None
        self.sent: list[list[float]] = []

    def start_prompt(self) -> str:
        return "Follower: ¿Continuar? (yes/no): "

    def check_start(self) -> list[str]:
        self.calls.append("core.check_start")
        return next(self._checks)

    def connect(self) -> None:
        self.calls.append("core.connect")
        self.is_connected = True

    def get_joints(self) -> list[float]:
        if self.get_error is not None:
            raise self.get_error
        return list(self.joints_rad)

    def send_joints(self, q_rad: list[float]) -> None:
        self.sent.append(list(q_rad))

    def disconnect(self) -> None:
        self.calls.append("core.disconnect")
        self.is_connected = False


class FakeCamera:
    def __init__(self, name: str, calls: list[str], fail: str = "") -> None:
        self.name = name
        self.calls = calls
        self.fail = fail
        self.height, self.width = 48, 64
        self.is_connected = False

    def connect(self) -> None:
        self.calls.append(f"{self.name}.connect")
        if self.fail == "connect":
            raise RuntimeError(f"{self.name}: no se pudo abrir")
        self.is_connected = True

    def read_latest(self) -> np.ndarray:
        return np.zeros((self.height, self.width, 3), dtype=np.uint8)

    def disconnect(self) -> None:
        self.calls.append(f"{self.name}.disconnect")
        if self.fail == "disconnect":
            raise RuntimeError(f"{self.name}: fallo al cerrar")
        self.is_connected = False


def answers(*replies: str) -> Callable[[str], str]:
    it = iter(replies)
    return lambda _prompt: next(it)


def make_robot(tmp_path: Path, **config: Any) -> UrFollower:
    return UrFollower(UrFollowerConfig(config_path=TEMPLATE, calibration_dir=tmp_path, **config))


@pytest.fixture
def calls() -> list[str]:
    return []


@pytest.fixture
def robot(tmp_path: Path, calls: list[str]) -> UrFollower:
    """Robot con núcleo y una cámara de mentira, que contesta `yes`."""
    robot = make_robot(tmp_path)
    robot.core = FakeCore(calls)
    robot.cameras = {"front": FakeCamera("front", calls)}
    robot.ask = answers("yes")
    return robot


# --- features y config ---------------------------------------------------------------------


def test_features_without_connecting(tmp_path: Path) -> None:
    camera = OpenCVCameraConfig(index_or_path=0, width=640, height=480, fps=30)
    robot = make_robot(tmp_path, cameras={"front": camera})
    assert robot.action_features == dict.fromkeys(JOINT_KEYS, float)
    assert robot.observation_features == {**dict.fromkeys(JOINT_KEYS, float), "front": (480, 640, 3)}
    assert list(robot.cameras) == ["front"]  # `lerobot-record` lo usa sin `hasattr`
    assert not robot.is_connected


def test_missing_yaml_raises_config_error_with_absolute_path(tmp_path: Path) -> None:
    with pytest.raises(ConfigError) as exc_info:
        UrFollower(UrFollowerConfig(config_path=Path("no_existe.yaml"), calibration_dir=tmp_path))
    assert str(Path("no_existe.yaml").absolute()) in str(exc_info.value)


def test_nothing_to_calibrate(robot: UrFollower, calls: list[str]) -> None:
    assert robot.is_calibrated
    robot.calibrate()
    robot.configure()
    assert calls == []


# --- connect -------------------------------------------------------------------------------


def test_connect_asks_checks_arms_and_then_opens_cameras(robot: UrFollower, calls: list[str]) -> None:
    prompts: list[str] = []
    robot.ask = lambda prompt: prompts.append(prompt) or "yes"
    robot.connect()
    assert prompts == [robot.core.start_prompt()]
    assert calls == ["core.check_start", "core.connect", "front.connect"]
    assert robot.is_connected


def test_connect_asks_again_while_checks_fail(robot: UrFollower, calls: list[str]) -> None:
    robot.core = FakeCore(calls, checks=[["follower base: 3,0° fuera de tolerancia"], []])
    robot.ask = answers("yes", "yes")
    robot.connect()
    assert calls == ["core.check_start", "core.check_start", "core.connect", "front.connect"]


def test_connect_answered_no_raises_keyboard_interrupt_without_arming(
    robot: UrFollower, calls: list[str], caplog: pytest.LogCaptureFixture
) -> None:
    robot.ask = answers("no")
    with caplog.at_level(logging.INFO), pytest.raises(KeyboardInterrupt):
        robot.connect(calibrate=False)  # así lo llama `lerobot-calibrate`
    assert "arranque cancelado por el operador" in caplog.text
    assert calls == []
    assert not robot.is_connected


def test_camera_failure_after_arming_disarms_the_follower(robot: UrFollower, calls: list[str]) -> None:
    robot.cameras = {"front": FakeCamera("front", calls, fail="connect")}
    with pytest.raises(RuntimeError, match="no se pudo abrir"):
        robot.connect()
    assert calls == ["core.check_start", "core.connect", "front.connect", "core.disconnect"]
    assert not robot.is_connected


# --- teleoperación -------------------------------------------------------------------------


def test_get_observation_has_measured_joints_and_camera_frames(robot: UrFollower) -> None:
    robot.connect()
    obs = robot.get_observation()
    assert set(obs) == set(robot.observation_features)
    assert [obs[key] for key in JOINT_KEYS] == pytest.approx(START_Q_RAD)
    assert obs["front"].shape == (48, 64, 3)


def test_get_observation_propagates_a_follower_stop(robot: UrFollower) -> None:
    robot.connect()
    robot.core.get_error = FollowerStoppedError("el follower está parado: parada de seguridad")
    with pytest.raises(FollowerStoppedError):
        robot.get_observation()
    assert robot.is_connected  # hasta disconnect(), aunque el follower ya esté parado


def test_send_action_sends_joints_in_ur_order_and_returns_them(robot: UrFollower) -> None:
    robot.connect()
    action = {key: np.float32(i / 10) for i, key in reversed(list(enumerate(JOINT_KEYS)))}
    sent = robot.send_action(action)
    assert robot.core.sent == [pytest.approx([0.0, 0.1, 0.2, 0.3, 0.4, 0.5])]
    assert list(sent) == JOINT_KEYS and all(type(v) is float for v in sent.values())


# --- disconnect ----------------------------------------------------------------------------


def test_disconnect_closes_cameras_and_core(robot: UrFollower, calls: list[str]) -> None:
    robot.connect()
    calls.clear()
    robot.disconnect()
    assert calls == ["front.disconnect", "core.disconnect"]
    assert not robot.is_connected


def test_disconnect_twice_or_without_connect_does_not_raise(robot: UrFollower, calls: list[str]) -> None:
    robot.disconnect()
    robot.connect()
    robot.disconnect()
    robot.disconnect()
    assert calls.count("front.disconnect") == 1


def test_disconnect_closes_the_core_and_does_not_raise_when_a_camera_fails(
    robot: UrFollower, calls: list[str], caplog: pytest.LogCaptureFixture
) -> None:
    robot.cameras = {"front": FakeCamera("front", calls, fail="disconnect")}
    robot.connect()
    robot.disconnect()
    assert "core.disconnect" in calls
    assert "fallo al cerrar" in caplog.text


# --- con el núcleo real --------------------------------------------------------------------


def test_with_the_real_core_arms_moves_reads_and_stops(tmp_path: Path) -> None:
    rig = Rig()
    robot = make_robot(tmp_path)
    robot.core = UrFollowerCore(
        load_config(TEMPLATE), launch=rig.launch, rtde_factory=lambda _ip: FakeRTDE(packets(1), hz=1000)
    )
    robot.ask = answers("yes")
    robot.connect()
    try:
        target = {
            key: q + (math.radians(0.5) if key == "base.pos" else 0.0)
            for key, q in zip(JOINT_KEYS, START_Q_RAD, strict=True)
        }
        robot.send_action(target)
        assert [robot.get_observation()[key] for key in JOINT_KEYS] == pytest.approx(START_Q_RAD)
    finally:
        robot.disconnect()
    assert rig.worker is not None and not rig.worker.is_alive()
    assert "parada pedida" in robot.core.status().stop_reason
