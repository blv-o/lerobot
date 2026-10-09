"""Trazas RTDE grabadas en URSim, reproducidas por FakeRTDE: el bucle reacciona a datos reales.

Las graba plugins/tests/ursim/test_follower_ursim.py con UR_RECORD_TRACES=1. Cada paquete trae
los campos de la receta de salida del bucle tal como los mandó el controlador (URSim 5.25.2).
"""

import json
import math
from pathlib import Path
from typing import Any

import pytest
from fakes import FakeClock, FakeDashboard, FakeRTDE
from ur_core.config import FollowerConfig, ServoConfig, WatchdogConfig
from ur_core.streaming import FollowerState, SharedState, StreamingLoop

TRACES = Path(__file__).parents[1] / "traces"
HZ = 125
FOLLOWER = FollowerConfig(
    type="ur3e",
    ip="127.0.0.2",
    servo=ServoConfig(hz=HZ, max_joint_speed_rad_s=math.radians(60), target_period_s=0.033),
    watchdog=WatchdogConfig(hold_s=0.1, stop_s=0.5),
)
MAX_STEP_RAD = math.radians(60) / HZ


def load_trace(name: str) -> list[dict[str, Any]]:
    return json.loads((TRACES / f"{name}.json").read_text(encoding="utf-8"))


def first_change(trace: list[dict[str, Any]], field: str) -> int:
    return next(i for i in range(1, len(trace)) if trace[i][field] != trace[i - 1][field])


def replay(trace: list[dict[str, Any]], first_target: int) -> tuple[StreamingLoop, FakeRTDE, SharedState]:
    """Reproduce la traza mandando, cada 4 paquetes desde `first_target`, la pose grabada como
    consigna (lo que haría un leader que copia al follower)."""
    clock = FakeClock()
    shared = SharedState()

    def send(k: int) -> Any:
        return lambda: shared.write_target(trace[k]["actual_q"], clock())

    actions = {k: send(k) for k in range(first_target, len(trace), 4)}
    rtde = FakeRTDE(trace, clock=clock, hz=HZ, actions=actions)
    loop = StreamingLoop(
        FOLLOWER,
        math.radians(2),
        shared,
        rtde=rtde,
        dashboard=FakeDashboard(),
        clock=clock,
    )
    loop.serve()
    return loop, rtde, shared


@pytest.mark.parametrize("name", ["follower_sine", "follower_stopped_from_pendant"])
def test_arms_when_the_real_script_starts_beating(name: str) -> None:
    """En URSim el heartbeat viejo (del programa anterior) pasa a 0 al arrancar nuestro script."""
    trace = load_trace(name)
    script_start = first_change(trace, "output_int_register_0")
    loop, _, _ = replay(trace, first_target=script_start + 2)
    wait_events = [t for t, state, _ in loop.events if state == FollowerState.WAIT]
    assert len(wait_events) == 1
    assert trace[script_start]["runtime_state"] == 2  # PLAYING, como espera el bucle


def test_sine_session_runs_until_the_program_ends() -> None:
    trace = load_trace("follower_sine")
    end = max(i for i, p in enumerate(trace) if p["runtime_state"] == 2) + 1
    loop, rtde, shared = replay(trace, first_target=first_change(trace, "output_int_register_0") + 2)
    states = [state for _, state, _ in loop.events]
    assert states == [FollowerState.WAIT, FollowerState.RUN, FollowerState.STOP]
    assert "runtime_state=STOPPED" in shared.read_state()[1]
    assert rtde.received == end + 1  # para en el mismo paquete en que el programa deja de correr
    q = rtde.written_q()
    assert max(
        max(abs(b - a) for a, b in zip(q0, q1, strict=True)) for q0, q1 in zip(q, q[1:], strict=False)
    ) <= (MAX_STEP_RAD + 1e-12)


def test_program_stopped_from_pendant_stops_on_that_packet() -> None:
    trace = load_trace("follower_stopped_from_pendant")
    script_start = first_change(trace, "output_int_register_0")
    stopped = next(i for i in range(script_start, len(trace)) if trace[i]["runtime_state"] != 2)
    loop, rtde, shared = replay(trace, first_target=script_start + 1)
    assert FollowerState.RUN in [state for _, state, _ in loop.events]
    assert "runtime_state=STOPPED" in shared.read_state()[1]
    assert rtde.received == stopped + 1
    assert rtde.written_enable()[-1] == 0
