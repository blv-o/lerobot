"""Bucle de streaming del follower: arranque, estados WAIT/RUN/HOLD/STOP y cierre.

Todo corre en el hilo del test con `FakeClock`: cada `receive()` avanza 1/hz y las consignas
de LeRobot se inyectan como acciones antes de un paquete concreto. Nada duerme.
"""

import logging
import math
import multiprocessing
from collections.abc import Callable
from typing import Any

import pytest
from fakes import (
    RUNTIME_STOPPED,
    SAFETY_PROTECTIVE_STOP,
    SAFETY_REDUCED,
    START_Q_RAD,
    FakeClock,
    FakeDashboard,
    FakeRTDE,
    FakeSecondary,
    packet,
    packets,
)
from ur_core.config import FollowerConfig, ServoConfig, WatchdogConfig
from ur_core.follower_script import render_follower_script
from ur_core.streaming import ARM_TIMEOUT_S, FollowerState, SharedState, StreamingLoop

HZ = 125
FOLLOWER = FollowerConfig(
    type="ur3e",
    ip="127.0.0.2",
    servo=ServoConfig(
        hz=HZ,
        gain=300,
        lookahead_s=0.1,
        max_joint_speed_rad_s=math.radians(60),
        target_period_s=0.033,
    ),
    watchdog=WatchdogConfig(hold_s=0.1, stop_s=0.5),
)
TOL_RAD = math.radians(2)
MAX_STEP_RAD = math.radians(60) / HZ
ARM_PACKETS = 2  # el primero se lee al armar; en el segundo el heartbeat ya ha cambiado
EPS = 1e-12


def offset(q_rad: list[float], joint: int, delta_rad: float) -> list[float]:
    out = list(q_rad)
    out[joint] += delta_rad
    return out


class Rig:
    """Bucle + dobles de test. `actions[k](rig)` corre antes de entregar el paquete k."""

    def __init__(
        self,
        pkts: list[dict[str, Any] | None],
        actions: dict[int, Callable[["Rig"], None]] | None = None,
        fail: tuple[str, ...] = (),
        dashboard_fail: tuple[str, ...] = (),
    ) -> None:
        self.clock = FakeClock()
        self.shared = SharedState()
        self.parent_is_alive = True
        bound = {k: (lambda f=f: f(self)) for k, f in (actions or {}).items()}
        self.rtde = FakeRTDE(pkts, clock=self.clock, hz=HZ, actions=bound, fail=fail)
        self.dashboard = FakeDashboard(fail=dashboard_fail)
        self.secondary = FakeSecondary()
        self.sent_before_script: int | None = None
        self.loop = StreamingLoop(
            FOLLOWER,
            TOL_RAD,
            self.shared,
            rtde=self.rtde,
            dashboard=self.dashboard,
            send_script=self._send_script,
            clock=self.clock,
            parent_alive=lambda: self.parent_is_alive,
        )

    def _send_script(self, host: str, text: str) -> None:
        self.sent_before_script = len(self.rtde.sent)
        self.secondary(host, text)

    def target(self, q_rad: list[float]) -> None:
        """Lo que hará `UrFollowerCore.send_joints` en el padre."""
        self.shared.write_target(q_rad, self.clock())

    def serve(self) -> "Rig":
        self.loop.serve()
        return self

    @property
    def state(self) -> FollowerState:
        return self.shared.read_state()[0]

    @property
    def reason(self) -> str:
        return self.shared.read_state()[1]

    def states(self) -> list[FollowerState]:
        return [state for _, state, _ in self.loop.events]

    def steps(self) -> list[float]:
        q = self.rtde.written_q()
        return [
            max(abs(b - a) for a, b in zip(q0, q1, strict=True)) for q0, q1 in zip(q, q[1:], strict=False)
        ]


def send_at(q_rad: list[float]) -> Callable[[Rig], None]:
    return lambda rig: rig.target(q_rad)


# --- arranque ------------------------------------------------------------------------------


def test_arm_declares_recipes_with_explicit_types() -> None:
    rig = Rig(packets(3)).serve()
    names, types, frequency = rig.rtde.output_setup
    assert names == [
        "actual_q",
        "timestamp",
        "robot_mode",
        "safety_mode",
        "runtime_state",
        "output_int_register_0",
    ]
    assert len(types) == len(names) and frequency == HZ
    inputs = rig.rtde.input_setups
    assert inputs[0] == ([f"input_double_register_{i}" for i in range(6)], ["DOUBLE"] * 6)
    assert inputs[1] == (["input_int_register_0"], ["INT32"])


def test_arm_writes_current_pose_disabled_before_uploading_script() -> None:
    rig = Rig(packets(3)).serve()
    assert rig.sent_before_script is not None and rig.sent_before_script >= 1
    first = rig.rtde.sent[0][1]
    assert [first[f"input_double_register_{i}"] for i in range(6)] == START_Q_RAD
    assert rig.rtde.sent[1][1] == {"input_int_register_0": 0}
    assert rig.secondary.scripts == [(FOLLOWER.ip, render_follower_script(FOLLOWER.servo, FOLLOWER.watchdog))]


def test_arm_reaches_wait_when_heartbeat_changes() -> None:
    rig = Rig(packets(5)).serve()
    assert rig.states()[:1] == [FollowerState.WAIT]


def test_arm_fails_if_heartbeat_never_changes() -> None:
    frozen = [packet(START_Q_RAD, heartbeat=7) for _ in range(int(ARM_TIMEOUT_S * HZ) + 10)]
    rig = Rig(frozen).serve()
    assert rig.state == FollowerState.STOP
    assert "heartbeat" in rig.reason
    assert FollowerState.WAIT not in rig.states()
    assert 1 not in rig.rtde.written_enable()


@pytest.mark.parametrize(
    ("fail", "fragment"),
    [
        ("connect", "conectar"),
        ("send_output_setup", "salida"),
        ("send_input_setup", "entrada"),
        ("send_start", "iniciar"),
    ],
)
def test_rtde_setup_failures_stop_with_reason_and_no_script(fail: str, fragment: str) -> None:
    rig = Rig(packets(3), fail=(fail,)).serve()
    assert rig.state == FollowerState.STOP
    assert fragment in rig.reason
    assert rig.secondary.scripts == []
    # No hay programa nuestro corriendo: no se manda `stop` a un programa ajeno.
    assert "stop" not in rig.dashboard.commands


def test_arm_refuses_robot_not_running() -> None:
    rig = Rig(packets(3, robot_mode=5)).serve()  # IDLE: frenos puestos
    assert rig.state == FollowerState.STOP
    assert "robot_mode" in rig.reason
    assert rig.secondary.scripts == []


def test_arm_refuses_robot_in_protective_stop() -> None:
    rig = Rig(packets(3, safety_mode=SAFETY_PROTECTIVE_STOP)).serve()
    assert rig.state == FollowerState.STOP
    assert "PROTECTIVE_STOP" in rig.reason


# --- WAIT ----------------------------------------------------------------------------------


def test_wait_without_targets_holds_armed_pose_disabled() -> None:
    rig = Rig(packets(40)).serve()
    assert all(q == START_Q_RAD for q in rig.rtde.written_q())
    assert set(rig.rtde.written_enable()) == {0}
    assert FollowerState.RUN not in rig.states()


def test_wait_does_not_time_out_without_targets() -> None:
    """Entre conectar el follower y la primera consigna LeRobot conecta el leader y pregunta
    yes/no: puede pasar mucho más que stop_s sin que sea un fallo."""
    n = int(10 * FOLLOWER.watchdog.stop_s * HZ)
    rig = Rig(packets(n)).serve()
    assert "stream" in rig.reason  # solo para al acabarse los paquetes


def test_first_target_far_from_pose_stops_without_ever_enabling() -> None:
    far = offset(START_Q_RAD, 4, math.radians(5))
    rig = Rig(packets(20), actions={5: send_at(far)}).serve()
    assert rig.state == FollowerState.STOP
    assert "primera consigna lejos de la posición actual" in rig.reason
    assert 1 not in rig.rtde.written_enable()
    # El script nunca vio enable=1, así que no sale solo: lo para el Dashboard.
    assert "stop" in rig.dashboard.commands


def test_first_target_within_tolerance_enables_and_respects_step_limit() -> None:
    near = offset(START_Q_RAD, 0, math.radians(1.9))
    rig = Rig(packets(30), actions={5: send_at(near)}).serve()
    assert FollowerState.RUN in rig.states()
    assert 1 in rig.rtde.written_enable()
    assert max(rig.steps()) <= MAX_STEP_RAD + EPS


# --- RUN -----------------------------------------------------------------------------------


def test_large_step_never_exceeds_speed_limit() -> None:
    near = offset(START_Q_RAD, 0, math.radians(1))
    jump = offset(START_Q_RAD, 0, math.pi)
    actions = {5: send_at(near), 10: send_at(jump)}
    actions.update({k: send_at(jump) for k in range(14, 200, 4)})  # se sigue mandando
    rig = Rig(packets(200), actions=actions).serve()
    assert max(rig.steps()) <= MAX_STEP_RAD + EPS


def test_regular_targets_give_continuous_output_that_reaches_each_target() -> None:
    """Consignas cada 33 ms (≈ 4 ciclos de 8 ms) con incrementos pequeños: la salida avanza
    sin saltos y alcanza cada consigna antes de que llegue la siguiente."""
    targets = [offset(START_Q_RAD, 0, 0.002 * k) for k in range(1, 30)]
    first_packet = 5
    times = [first_packet + round(k * 0.033 * HZ) for k in range(len(targets))]
    actions = {t: send_at(q) for t, q in zip(times, targets, strict=True)}
    rig = Rig(packets(times[-1] + 30), actions=actions).serve()
    written = [q[0] for q in rig.rtde.written_q()]
    assert max(rig.steps()) <= 0.002 * (1 / HZ) / 0.033 * 1.6
    assert written[-1] == pytest.approx(targets[-1][0])


def test_early_target_starts_from_last_written_command() -> None:
    """Una consigna que llega antes de terminar la anterior parte de donde va el robot."""
    a = offset(START_Q_RAD, 0, 0.01)
    b = offset(START_Q_RAD, 0, 0.02)
    rig = Rig(packets(30), actions={5: send_at(a), 7: send_at(b)}).serve()
    written = [q[0] for q in rig.rtde.written_q()]
    jumps = [abs(y - x) for x, y in zip(written, written[1:], strict=False)]
    # Sin partir del último comando habría un salto hacia `a` al llegar `b`.
    assert max(jumps) <= (0.02 - 0.0) * (1 / HZ) / 0.033 + EPS


# --- HOLD y paradas por edad -------------------------------------------------------------------


def test_hold_repeats_last_command_and_returns_to_run_on_new_target() -> None:
    q1 = offset(START_Q_RAD, 0, 0.01)
    q2 = offset(START_Q_RAD, 0, 0.012)
    hold_cycles = math.ceil(FOLLOWER.watchdog.hold_s * HZ) + 3
    rig = Rig(packets(5 + hold_cycles + 20), actions={5: send_at(q1), 5 + hold_cycles: send_at(q2)}).serve()
    states = rig.states()
    i_hold = states.index(FollowerState.HOLD)
    assert states[i_hold + 1] == FollowerState.RUN
    # Mientras está en HOLD el robot está quieto en la última consigna.
    t_hold_ns, t_run_ns = rig.loop.events[i_hold][0], rig.loop.events[i_hold + 1][0]
    held = [
        [fields[f"input_double_register_{i}"] for i in range(6)]
        for t_ns, fields in rig.rtde.sent
        if t_hold_ns <= t_ns < t_run_ns and "input_double_register_0" in fields
    ]
    assert len(held) >= 2
    assert all(q == pytest.approx(q1) for q in held)


def test_stop_when_targets_stop_arriving_for_stop_time() -> None:
    q1 = offset(START_Q_RAD, 0, 0.01)
    rig = Rig(packets(200), actions={5: send_at(q1)}).serve()
    assert rig.state == FollowerState.STOP
    assert "sin consignas" in rig.reason
    _, t_target_ns, _ = rig.shared.read_target()  # la edad se cuenta desde que se envió
    t_stop_ns = next(t for t, s, _ in rig.loop.events if s == FollowerState.STOP)
    stop_after_s = (t_stop_ns - t_target_ns) / 1e9
    assert FOLLOWER.watchdog.stop_s <= stop_after_s <= FOLLOWER.watchdog.stop_s + 2 / HZ
    assert rig.rtde.written_enable()[-1] == 0
    assert rig.dashboard.commands[-1] == "stop"


# --- paradas por el robot -----------------------------------------------------------------------


def _run_then(pkts_after: list[dict[str, Any]]) -> Rig:
    """Arma, entra en RUN con una consigna cercana y sigue con `pkts_after` (consigna fresca)."""
    near = offset(START_Q_RAD, 0, 0.005)
    head = packets(10)
    hb0 = len(head)
    tail = [{**p, "output_int_register_0": hb0 + i} for i, p in enumerate(pkts_after)]
    actions = {k: send_at(near) for k in range(5, len(head) + len(tail), 4)}
    return Rig(head + tail, actions=actions).serve()


def test_protective_stop_stops_with_reason() -> None:
    rig = _run_then(packets(3) + packets(3, safety_mode=SAFETY_PROTECTIVE_STOP))
    assert rig.state == FollowerState.STOP
    assert "PROTECTIVE_STOP" in rig.reason


def test_reduced_mode_keeps_running() -> None:
    rig = _run_then(packets(30, safety_mode=SAFETY_REDUCED))
    assert FollowerState.STOP not in rig.states()[:-1]
    assert "stream" in rig.reason  # solo paró al acabarse los paquetes


def test_robot_leaving_running_mode_stops() -> None:
    rig = _run_then(packets(3, robot_mode=5))
    assert "robot_mode" in rig.reason


def test_program_stopped_from_pendant_stops() -> None:
    rig = _run_then(packets(3, runtime_state=RUNTIME_STOPPED))
    assert "runtime_state" in rig.reason


def test_frozen_heartbeat_stops_after_stop_time() -> None:
    near = offset(START_Q_RAD, 0, 0.005)
    frozen = [packet(START_Q_RAD, heartbeat=999) for _ in range(200)]
    actions = {k: send_at(near) for k in range(5, 210, 4)}  # consignas frescas: solo falla el heartbeat
    rig = Rig(packets(10) + frozen, actions=actions).serve()
    assert "heartbeat" in rig.reason
    assert len(rig.rtde.written_q()) < 10 + math.ceil(FOLLOWER.watchdog.stop_s * HZ) + 5


def test_receive_timeout_counts_as_lost_stream() -> None:
    rig = Rig([*packets(5), None, *packets(5)]).serve()
    assert "stream" in rig.reason
    assert rig.rtde.received == 6


# --- paradas pedidas y cierre ---------------------------------------------------------------------


def test_parent_death_stops_on_next_cycle() -> None:
    def kill_parent(rig: Rig) -> None:
        rig.parent_is_alive = False

    rig = Rig(packets(50), actions={10: kill_parent}).serve()
    assert "proceso padre" in rig.reason
    assert rig.rtde.received == 11


def test_stop_request_disables_and_sends_dashboard_stop() -> None:
    near = offset(START_Q_RAD, 0, 0.005)
    rig = Rig(packets(50), actions={5: send_at(near), 10: lambda rig: rig.shared.request_stop()}).serve()
    assert "parada pedida" in rig.reason
    assert rig.rtde.written_enable()[-1] == 0
    assert rig.dashboard.commands == ["stop"]
    assert not rig.rtde.is_connected()


def test_unexpected_exception_goes_through_the_same_shutdown(caplog: pytest.LogCaptureFixture) -> None:
    near = offset(START_Q_RAD, 0, 0.005)

    def boom(rig: Rig) -> None:
        raise RuntimeError("boom")

    with caplog.at_level(logging.ERROR):
        rig = Rig(packets(50), actions={5: send_at(near), 12: boom}).serve()
    assert rig.state == FollowerState.STOP
    assert "boom" in rig.reason
    assert rig.rtde.written_enable()[-1] == 0
    assert rig.dashboard.commands == ["stop"]
    assert "boom" in caplog.text  # registrado con traza, no silenciado


def test_dashboard_failure_on_shutdown_is_logged_and_state_still_published(
    caplog: pytest.LogCaptureFixture,
) -> None:
    with caplog.at_level(logging.ERROR):
        rig = Rig(packets(20), dashboard_fail=("send",)).serve()
    assert rig.state == FollowerState.STOP
    assert "Dashboard" in caplog.text


# --- memoria compartida y observabilidad -----------------------------------------------------------


def test_measured_pose_and_period_stats_are_published() -> None:
    moved = offset(START_Q_RAD, 3, 0.1)
    rig = Rig([*packets(10), *packets(HZ + 10, q_rad=moved)]).serve()
    assert rig.shared.read_measured() == pytest.approx(moved)
    p50_s, p99_s, max_s = rig.shared.read_period_stats()
    assert (
        p50_s == pytest.approx(1 / HZ) and p99_s == pytest.approx(1 / HZ) and max_s == pytest.approx(1 / HZ)
    )


def test_every_transition_is_recorded_with_monotonic_ns() -> None:
    near = offset(START_Q_RAD, 0, 0.005)
    rig = Rig(packets(200), actions={5: send_at(near)}).serve()
    assert rig.states() == [FollowerState.WAIT, FollowerState.RUN, FollowerState.HOLD, FollowerState.STOP]
    times = [t for t, _, _ in rig.loop.events]
    assert times == sorted(times)
    assert rig.loop.events[-1][2] == rig.reason


def test_new_target_is_detected_by_sequence_not_by_timestamp() -> None:
    """Dos consignas con el mismo t_ns (llegan entre dos paquetes) cuentan las dos."""
    shared = SharedState()
    shared.write_target([0.1] * 6, 5)
    seq1, _, _ = shared.read_target()
    shared.write_target([0.2] * 6, 5)
    seq2, t_ns, q = shared.read_target()
    assert seq2 == seq1 + 1 and t_ns == 5 and q == pytest.approx([0.2] * 6)


def _child_writes(shared: SharedState) -> None:
    shared.write_measured([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    shared.publish_state(FollowerState.STOP, "motivo con tildes: parada pedida ñ")


def test_shared_state_crosses_a_spawned_process() -> None:
    """Las conexiones nunca cruzan procesos, pero la memoria compartida sí (spawn, como en Windows)."""
    ctx = multiprocessing.get_context("spawn")
    shared = SharedState()
    child = ctx.Process(target=_child_writes, args=(shared,), daemon=True)
    child.start()
    child.join(timeout=30)
    assert child.exitcode == 0
    assert shared.read_measured() == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert shared.read_state() == (FollowerState.STOP, "motivo con tildes: parada pedida ñ")
