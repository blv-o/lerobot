"""Adaptador `UrLeader(Teleoperator)`: solo traduce entre LeRobot y `UrLeaderCore`."""

import logging
from collections.abc import Callable
from pathlib import Path

import pytest
from fakes import START_Q_RAD, FakeRTDE
from lerobot_teleoperator_ur_leader import UrLeader, UrLeaderConfig
from test_leader_core import ticking_packets
from ur_core import JOINT_NAMES, ConfigError, LeaderReadError, UrLeaderCore, load_config

TEMPLATE = Path(__file__).parents[2] / "configs" / "ur_config.yaml"
JOINT_KEYS = [f"{joint}.pos" for joint in JOINT_NAMES]


class FakeCore:
    """`UrLeaderCore` de mentira: anota en `calls` cada llamada."""

    def __init__(self, checks: list[list[str]] | None = None) -> None:
        self.calls: list[str] = []
        self._checks = iter(checks or [[]])
        self.is_connected = False
        self.read_error: Exception | None = None

    def start_prompt(self) -> str:
        return "Leader: ¿Continuar? (yes/no): "

    def check_start(self) -> list[str]:
        self.calls.append("check_start")
        return next(self._checks)

    def connect(self) -> None:
        self.calls.append("connect")
        self.is_connected = True

    def read_joints(self) -> list[float]:
        if self.read_error is not None:
            raise self.read_error
        return list(START_Q_RAD)

    def disconnect(self) -> None:
        self.calls.append("disconnect")
        self.is_connected = False


def answers(*replies: str) -> Callable[[str], str]:
    it = iter(replies)
    return lambda _prompt: next(it)


def make_leader(tmp_path: Path) -> UrLeader:
    return UrLeader(UrLeaderConfig(config_path=TEMPLATE, calibration_dir=tmp_path))


@pytest.fixture
def leader(tmp_path: Path) -> UrLeader:
    leader = make_leader(tmp_path)
    leader.core = FakeCore()
    leader.ask = answers("yes")
    return leader


def test_features_without_connecting(tmp_path: Path) -> None:
    leader = make_leader(tmp_path)
    assert leader.action_features == dict.fromkeys(JOINT_KEYS, float)
    assert leader.feedback_features == {}
    assert not leader.is_connected


def test_missing_yaml_raises_config_error_with_absolute_path(tmp_path: Path) -> None:
    with pytest.raises(ConfigError) as exc_info:
        UrLeader(UrLeaderConfig(config_path=Path("no_existe.yaml"), calibration_dir=tmp_path))
    assert str(Path("no_existe.yaml").absolute()) in str(exc_info.value)


def test_nothing_to_calibrate(leader: UrLeader) -> None:
    assert leader.is_calibrated
    leader.calibrate()
    leader.configure()
    assert leader.core.calls == []


def test_connect_asks_checks_and_connects(leader: UrLeader) -> None:
    prompts: list[str] = []
    leader.ask = lambda prompt: prompts.append(prompt) or "yes"
    leader.connect()
    assert prompts == [leader.core.start_prompt()]
    assert leader.core.calls == ["check_start", "connect"]
    assert leader.is_connected


def test_connect_answered_no_raises_keyboard_interrupt_without_connecting(
    leader: UrLeader, caplog: pytest.LogCaptureFixture
) -> None:
    leader.ask = answers("no")
    with caplog.at_level(logging.INFO), pytest.raises(KeyboardInterrupt):
        leader.connect(calibrate=False)
    assert "arranque cancelado por el operador" in caplog.text
    assert leader.core.calls == []
    assert not leader.is_connected


def test_get_action_returns_the_leader_joints(leader: UrLeader) -> None:
    leader.connect()
    action = leader.get_action()
    assert list(action) == JOINT_KEYS
    assert list(action.values()) == pytest.approx(START_Q_RAD)


def test_get_action_propagates_leader_read_error(leader: UrLeader) -> None:
    leader.connect()
    leader.core.read_error = LeaderReadError("el leader no da datos nuevos")
    with pytest.raises(LeaderReadError):
        leader.get_action()
    assert leader.is_connected  # hasta disconnect()


def test_send_feedback_sends_nothing_to_the_leader(leader: UrLeader) -> None:
    leader.connect()
    leader.send_feedback({})
    assert leader.core.calls == ["check_start", "connect"]


def test_disconnect_twice_or_without_connect_does_not_raise(leader: UrLeader) -> None:
    leader.disconnect()
    leader.connect()
    leader.disconnect()
    leader.disconnect()
    assert not leader.is_connected


def test_with_the_real_core_reads_and_stops(tmp_path: Path) -> None:
    leader = make_leader(tmp_path)
    connections: list[FakeRTDE] = []

    def rtde_factory(_ip: str) -> FakeRTDE:
        connections.append(FakeRTDE(ticking_packets(), hz=500))
        return connections[-1]

    leader.core = UrLeaderCore(load_config(TEMPLATE), rtde_factory=rtde_factory)
    leader.ask = answers("yes")
    leader.connect()
    try:
        assert list(leader.get_action().values()) == pytest.approx(START_Q_RAD)
    finally:
        leader.disconnect()
    assert not leader.is_connected
    assert all(con.input_setups == [] and con.sent == [] for con in connections)
