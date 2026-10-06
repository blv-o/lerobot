"""`UrFollowerCore` visto desde el padre: arranque, consignas, estado, parada y comprobaciones.

El bucle del robot ya está probado en test_streaming.py. Aquí corre en un hilo (o en un proceso
spawn real) con los dobles de test, y los tests esperan a un estado, nunca un tiempo fijo.
"""

import math
import multiprocessing
import threading
import time
from collections.abc import Callable, Iterator
from typing import Any

import pytest
import ur_core.follower as follower_module
import ur_core.streaming as streaming_module
from fakes import (
    START_Q_RAD,
    FakeDashboard,
    FakeRTDE,
    FakeSecondary,
    endless_packets,
    packets,
)
from ur_core.clock import now_ns
from ur_core.config import UR_TYPES, FollowerConfig, ServoConfig, TeleopConfig, UrLeaderConfig, WatchdogConfig
from ur_core.follower import (
    FOLLOWERS,
    FollowerStartError,
    FollowerStoppedError,
    UrFollowerCore,
)
from ur_core.streaming import FollowerState, SharedState, StreamingLoop

CONFIG = TeleopConfig(
    leader=UrLeaderConfig(type="ur3e", ip="127.0.0.3", rtde_hz=500, timeout_s=0.1),
    follower=FollowerConfig(
        type="ur3e",
        ip="127.0.0.2",
        servo=ServoConfig(
            hz=125, gain=300, lookahead_s=0.1, max_joint_speed_rad_s=math.radians(60), target_period_s=0.033
        ),
        watchdog=WatchdogConfig(hold_s=0.1, stop_s=0.5),
    ),
    start_pose_rad=tuple(START_Q_RAD),
    start_tolerance_rad=math.radians(2),
)
NEAR = [START_Q_RAD[0] + 0.005, *START_Q_RAD[1:]]
FAR = [START_Q_RAD[0] + math.radians(10), *START_Q_RAD[1:]]
WAIT_S = 10.0


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
        loop = StreamingLoop(
            follower, tol_rad, shared, rtde=self.rtde, dashboard=self.dashboard, send_script=FakeSecondary()
        )
        self.worker = ThreadWorker(target=loop.serve, daemon=True)
        self.worker.start()
        return self.worker


def wait_for(core: UrFollowerCore, state: FollowerState) -> None:
    deadline = time.monotonic() + WAIT_S
    while core.status().state != state:
        assert time.monotonic() < deadline, f"no llegó a {state.name}: {core.status()}"
        time.sleep(0.005)


@pytest.fixture
def rig() -> Iterator[Rig]:
    rig = Rig()
    yield rig
    if rig.worker is not None and rig.worker.is_alive():
        raise AssertionError("el test dejó el bucle del follower vivo")


# --- arranque ------------------------------------------------------------------------------


def test_connect_returns_once_armed_in_wait(rig: Rig) -> None:
    core = UrFollowerCore(CONFIG, launch=rig.launch)
    core.connect()
    assert core.is_connected
    assert core.status().state == FollowerState.WAIT
    assert core.get_joints() == pytest.approx(START_Q_RAD)
    core.disconnect()


def test_connect_raises_with_child_reason_when_arming_fails() -> None:
    rig = Rig(pkts=lambda: iter(packets(5, robot_mode=5)))
    core = UrFollowerCore(CONFIG, launch=rig.launch)
    with pytest.raises(FollowerStartError, match="robot_mode"):
        core.connect()
    assert not core.is_connected
    assert not rig.worker.is_alive()


def test_connect_raises_if_worker_dies_without_publishing() -> None:
    def launch(*_args: Any) -> ThreadWorker:
        worker = ThreadWorker(target=lambda: None, daemon=True)
        worker.start()
        return worker

    core = UrFollowerCore(CONFIG, launch=launch)
    with pytest.raises(FollowerStartError, match="terminó"):
        core.connect()


def test_connect_gives_up_and_terminates_a_stuck_worker(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(follower_module, "ARM_TIMEOUT_S", 0.05)
    monkeypatch.setattr(follower_module, "CONNECT_MARGIN_S", 0.05)
    release = threading.Event()
    workers: list[ThreadWorker] = []

    def launch(*_args: Any) -> ThreadWorker:
        worker = ThreadWorker(target=release.wait, daemon=True)
        worker.start()
        workers.append(worker)
        return worker

    core = UrFollowerCore(CONFIG, launch=launch)
    with pytest.raises(FollowerStartError, match="no quedó armado"):
        core.connect()
    assert workers[0].terminated
    release.set()


@pytest.mark.timeout(10)
def test_connect_gives_up_if_the_process_died_holding_the_shared_memory_lock(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(streaming_module, "LOCK_TIMEOUT_S", 0.05)

    def launch(_follower: FollowerConfig, _tol_rad: float, shared: SharedState) -> ThreadWorker:
        assert shared._lock.acquire(block=False)  # murió con el Lock cogido
        worker = ThreadWorker(target=lambda: None, daemon=True)
        worker.start()
        worker.join()
        return worker

    core = UrFollowerCore(CONFIG, launch=launch)
    with pytest.raises(FollowerStartError, match="memoria compartida"):
        core.connect()
    assert not core.is_connected


def test_connect_twice_does_not_launch_a_second_worker(rig: Rig) -> None:
    launches: list[int] = []

    def launch(*args: Any) -> ThreadWorker:
        launches.append(1)
        return rig.launch(*args)

    core = UrFollowerCore(CONFIG, launch=launch)
    core.connect()
    core.connect()
    assert launches == [1]
    core.disconnect()


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


def test_send_joints_is_non_blocking_and_reaches_the_robot(rig: Rig) -> None:
    core = UrFollowerCore(CONFIG, launch=rig.launch)
    core.connect()
    t0_ns = now_ns()
    core.send_joints(NEAR)
    assert now_ns() - t0_ns < 1_000_000  # < 1 ms
    wait_for(core, FollowerState.RUN)
    core.disconnect()
    assert 1 in rig.rtde.written_enable()


def test_send_joints_after_stop_raises_with_reason(rig: Rig) -> None:
    core = UrFollowerCore(CONFIG, launch=rig.launch)
    core.connect()
    core.send_joints(FAR)
    wait_for(core, FollowerState.STOP)
    with pytest.raises(FollowerStoppedError, match="primera consigna lejos"):
        core.send_joints(NEAR)
    core.disconnect()


def test_send_and_get_joints_raise_if_the_process_died_without_publishing_stop() -> None:
    """Un proceso muerto sin pasar por su cierre (EDR, `terminate()`, fallo nativo) deja el estado
    en RUN: sin mirar si vive, LeRobot seguiría mandando consignas y grabando una pose congelada."""
    workers: list[ThreadWorker] = []

    def launch(_follower: FollowerConfig, _tol_rad: float, shared: SharedState) -> ThreadWorker:
        def reach_run_and_die() -> None:
            shared.try_write_measured(START_Q_RAD)
            shared.publish_state(FollowerState.WAIT)
            while (shared.try_read_target() or (0,))[0] == 0:
                time.sleep(0.001)
            shared.publish_state(FollowerState.RUN)

        worker = ThreadWorker(target=reach_run_and_die, daemon=True)
        worker.start()
        workers.append(worker)
        return worker

    core = UrFollowerCore(CONFIG, launch=launch)
    core.connect()
    core.send_joints(NEAR)
    workers[0].join(timeout=WAIT_S)
    assert core.status().state == FollowerState.RUN
    with pytest.raises(FollowerStoppedError, match="terminó sin publicar el motivo"):
        core.send_joints(NEAR)
    with pytest.raises(FollowerStoppedError, match="terminó sin publicar el motivo"):
        core.get_joints()
    core.disconnect()


def test_get_joints_after_the_process_ended_raises_with_its_reason(rig: Rig) -> None:
    core = UrFollowerCore(CONFIG, launch=rig.launch)
    core.connect()
    core.send_joints(FAR)
    rig.worker.join(timeout=WAIT_S)
    with pytest.raises(FollowerStoppedError, match="primera consigna lejos"):
        core.get_joints()
    core.disconnect()


# --- parada --------------------------------------------------------------------------------


def test_disconnect_stops_the_loop_through_the_shutdown(rig: Rig) -> None:
    core = UrFollowerCore(CONFIG, launch=rig.launch)
    core.connect()
    core.send_joints(NEAR)
    wait_for(core, FollowerState.RUN)
    core.disconnect()
    assert not rig.worker.is_alive()
    assert not rig.worker.terminated
    assert not core.is_connected
    status = core.status()
    assert status.state == FollowerState.STOP and "parada pedida" in status.stop_reason
    assert rig.rtde.written_enable()[-1] == 0
    assert rig.dashboard.commands == ["stop"]


def test_disconnect_twice_or_without_connect_does_nothing(rig: Rig) -> None:
    UrFollowerCore(CONFIG, launch=rig.launch).disconnect()
    core = UrFollowerCore(CONFIG, launch=rig.launch)
    core.connect()
    core.disconnect()
    core.disconnect()


def test_status_reports_period_statistics(rig: Rig) -> None:
    core = UrFollowerCore(CONFIG, launch=rig.launch)
    core.connect()
    core.send_joints(NEAR)
    deadline = time.monotonic() + WAIT_S
    while core.status().period_max_s == 0:
        assert time.monotonic() < deadline
        core.send_joints(NEAR)
        time.sleep(0.02)
    core.disconnect()
    status = core.status()
    assert 0 < status.period_p50_s <= status.period_p99_s <= status.period_max_s


# --- proceso real ----------------------------------------------------------------------------


def _fake_streaming_main(follower: FollowerConfig, tol_rad: float, shared: SharedState) -> None:
    """Como `streaming_main`, pero con dobles de test, en un proceso hijo de verdad."""
    StreamingLoop(
        follower,
        tol_rad,
        shared,
        rtde=FakeRTDE(endless_packets(), hz=follower.servo.hz),
        dashboard=FakeDashboard(),
        send_script=FakeSecondary(),
        parent_alive=multiprocessing.parent_process().is_alive,
    ).serve()


def test_real_spawned_process_arms_runs_and_stops_on_disconnect() -> None:
    processes: list[multiprocessing.Process] = []

    def launch(follower: FollowerConfig, tol_rad: float, shared: SharedState) -> multiprocessing.Process:
        process = multiprocessing.get_context("spawn").Process(
            target=_fake_streaming_main, args=(follower, tol_rad, shared), daemon=True
        )
        process.start()
        processes.append(process)
        return process

    core = UrFollowerCore(CONFIG, launch=launch)
    core.connect()
    core.send_joints(NEAR)
    wait_for(core, FollowerState.RUN)
    core.disconnect()
    assert processes[0].exitcode == 0
    assert "parada pedida" in core.status().stop_reason


# --- comprobaciones de arranque --------------------------------------------------------------


def make_core(
    remote: str = "true", q_rad: list[float] = START_Q_RAD, **fail: tuple[str, ...]
) -> UrFollowerCore:
    def rtde_factory(_ip: str) -> FakeRTDE:
        return FakeRTDE(packets(1, q_rad=q_rad), hz=1000, fail=fail.get("rtde", ()))

    def dashboard_factory(_ip: str) -> FakeDashboard:
        return FakeDashboard({"is in remote control": remote}, fail=fail.get("dashboard", ()))

    return UrFollowerCore(CONFIG, rtde_factory=rtde_factory, dashboard_factory=dashboard_factory)


def test_check_start_ok_in_remote_control_and_start_pose() -> None:
    assert make_core().check_start() == []


def test_check_start_reports_local_mode() -> None:
    errors = make_core(remote="false").check_start()
    assert len(errors) == 1 and "Remote Control" in errors[0]


def test_check_start_reports_joint_out_of_tolerance() -> None:
    q = list(START_Q_RAD)
    q[4] += math.radians(5.3)
    assert make_core(q_rad=q).check_start() == ["follower wrist_2: 5,3° fuera de tolerancia"]


def test_check_start_reports_unreachable_dashboard_and_rtde() -> None:
    errors = make_core(dashboard=("connect",), rtde=("connect",)).check_start()
    assert len(errors) == 2
    assert "Dashboard" in errors[0] and "RTDE" in errors[1]


def test_start_prompt_tells_what_to_prepare_on_the_pendant() -> None:
    prompt = UrFollowerCore(CONFIG).start_prompt()
    assert "Remote Control" in prompt
    assert "[0, -90, 90, -90, -90, 0]" in prompt and "± 2" in prompt
    assert "yes/no" in prompt


def test_every_accepted_ur_model_maps_to_the_core() -> None:
    assert set(FOLLOWERS) == set(UR_TYPES)
    assert set(FOLLOWERS.values()) == {UrFollowerCore}
