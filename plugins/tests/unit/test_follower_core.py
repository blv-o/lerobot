"""`UrFollowerCore`: consignas y comprobaciones de arranque."""

import dataclasses
import math
import threading
from collections.abc import Callable, Iterator
from types import SimpleNamespace
from typing import Any

import pytest
from fakes import (
    START_Q_RAD,
    FakeDashboard,
    FakeRTDE,
    endless_packets,
    packets,
)
from ur_core.config import FollowerConfig, ServoConfig, TeleopConfig, UrLeaderConfig, WatchdogConfig
from ur_core.follower import UrFollowerCore
from ur_core.streaming import FollowerState, SharedState, StreamingLoop

CONFIG = TeleopConfig(
    leader=UrLeaderConfig(type="ur3e", ip="127.0.0.3", rtde_hz=500, timeout_s=0.1),
    follower=FollowerConfig(
        type="ur3e",
        ip="127.0.0.2",
        servo=ServoConfig(hz=125, max_joint_speed_rad_s=math.radians(60), target_period_s=0.033),
        watchdog=WatchdogConfig(hold_s=0.1, stop_s=0.5),
    ),
    start_pose_rad=tuple(START_Q_RAD),
    start_tolerance_rad=math.radians(2),
)
NEAR = [START_Q_RAD[0] + 0.005, *START_Q_RAD[1:]]


class ThreadWorker(threading.Thread):
    """Un hilo con la interfaz de `multiprocessing.Process` que usa `UrFollowerCore`."""

    terminated = False

    def terminate(self) -> None:
        self.terminated = True


class Rig:
    def __init__(self, pkts: Callable[[], Any] = endless_packets) -> None:
        self.dashboard = FakeDashboard()
        self.rtde: FakeRTDE | None = None
        self.worker: ThreadWorker | None = None
        self._pkts = pkts

    def launch(self, follower: FollowerConfig, tol_rad: float, shared: SharedState) -> ThreadWorker:
        self.rtde = FakeRTDE(self._pkts(), hz=follower.servo.hz)  # sin reloj falso: ritmo real
        loop = StreamingLoop(follower, tol_rad, shared, rtde=self.rtde, dashboard=self.dashboard)
        self.worker = ThreadWorker(target=loop.serve, daemon=True)
        self.worker.start()
        return self.worker


@pytest.fixture
def rig() -> Iterator[Rig]:
    rig = Rig()
    yield rig
    if rig.worker is not None and rig.worker.is_alive():
        raise AssertionError("el test dejó el bucle del follower vivo")


# --- consignas -----------------------------------------------------------------------------


@pytest.mark.parametrize(
    "bad",
    [
        START_Q_RAD[:5],
        [*START_Q_RAD, 0.0],
        [math.nan, *START_Q_RAD[1:]],
        [math.inf, *START_Q_RAD[1:]],
    ],
)
def test_send_joints_rejects_wrong_size_and_non_finite(rig: Rig, bad: list[float]) -> None:
    core = UrFollowerCore(CONFIG, launch=rig.launch)
    core.connect()
    with pytest.raises(ValueError):
        core.send_joints(bad)
    assert core.status().state == FollowerState.WAIT  # nunca llegó al robot
    core.disconnect()


def test_send_joints_before_connect_raises() -> None:
    core = UrFollowerCore(CONFIG, launch=Rig().launch)
    with pytest.raises(RuntimeError, match="no conectado"):
        core.send_joints(NEAR)


# --- comprobaciones de arranque --------------------------------------------------------------


def make_core(
    q_rad: list[float] = START_Q_RAD, config: TeleopConfig = CONFIG, rtde_fail: tuple[str, ...] = ()
) -> UrFollowerCore:
    def rtde_factory(_ip: str) -> FakeRTDE:
        return FakeRTDE(packets(1, q_rad=q_rad), hz=1000, fail=rtde_fail)

    return UrFollowerCore(config, rtde_factory=rtde_factory)


def test_check_start_ok_in_start_pose() -> None:
    assert make_core().check_start() == []


def test_check_start_reports_joint_out_of_tolerance() -> None:
    q = list(START_Q_RAD)
    q[4] += math.radians(5.3)
    assert make_core(q_rad=q).check_start() == ["follower wrist_2: 5,3° fuera de tolerancia"]


def test_check_start_reports_unreachable_rtde() -> None:
    errors = make_core(rtde_fail=("connect",)).check_start()
    assert len(errors) == 1 and "RTDE" in errors[0]


def test_check_start_reports_robot_not_answering_the_rtde_setup() -> None:
    """El cliente oficial lanza AttributeError si la respuesta a la receta no llega a tiempo."""
    errors = make_core(rtde_fail=("setup_timeout",)).check_start()
    assert len(errors) == 1
    assert "RTDE" in errors[0] and "no respondió" in errors[0]


def test_check_start_does_not_blame_the_robot_for_our_own_attribute_error() -> None:
    """Solo el AttributeError del cliente oficial significa "el robot no respondió"."""
    broken = dataclasses.replace(
        CONFIG, follower=dataclasses.replace(CONFIG.follower, servo=SimpleNamespace())
    )
    with pytest.raises(AttributeError):
        make_core(config=broken).check_start()


def test_check_start_closes_the_rtde_socket_when_negotiation_fails() -> None:
    """El cliente oficial abre el socket antes de negociar el protocolo: si falla, hay que cerrarlo."""
    connections: list[FakeRTDE] = []

    def rtde_factory(_ip: str) -> FakeRTDE:
        connections.append(FakeRTDE(packets(1), hz=1000, fail=("connect_protocol",)))
        return connections[-1]

    errors = UrFollowerCore(CONFIG, rtde_factory=rtde_factory).check_start()
    assert len(errors) == 1 and "RTDE" in errors[0]
    assert not connections[0].connected


def test_start_prompt_tells_what_to_prepare_on_the_pendant() -> None:
    prompt = UrFollowerCore(CONFIG).start_prompt()
    assert "Play" in prompt and "Remote" not in prompt
    assert "[0, -90, 90, -90, -90, 0]" in prompt and "± 2" in prompt
    assert "yes/no" in prompt
    assert prompt.endswith("\n")  # con `| Tee-Object`, sin él la pregunta no se ve
