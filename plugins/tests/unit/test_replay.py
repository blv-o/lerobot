"""Trazas RTDE grabadas en URSim, reproducidas por FakeRTDE: el follower reacciona a datos reales.

Las graban plugins/tests/ursim/test_follower_ursim.py y test_leader_ursim.py con
UR_RECORD_TRACES=1. Cada paquete trae los campos de la receta de salida tal como los mandó el
controlador (URSim 5.25.2).
"""

import itertools
import json
import math
import time
from pathlib import Path
from typing import Any

import pytest
from fakes import FakeClock, FakeRTDE
from ur_core.config import FollowerConfig, TeleopConfig, UrLeaderConfig
from ur_core.follower import FollowerStoppedError, UrFollowerCore
from ur_core.leader import LeaderReadError, UrLeaderCore

TRACES = Path(__file__).parents[1] / "traces"
HZ = 125
SEND_EVERY = 4  # LeRobot a 30 fps: una consigna cada ~4 paquetes
LEADER = UrLeaderConfig(type="ur3e", ip="127.0.0.3", timeout_s=0.1)
CONFIG = TeleopConfig(
    leader=LEADER,
    follower=FollowerConfig(type="ur3e", ip="127.0.0.2"),
    start_pose_rad=(0.0,) * 6,
    start_tolerance_rad=math.radians(2),
)


def load_trace(name: str) -> list[dict[str, Any]]:
    return json.loads((TRACES / f"{name}.json").read_text(encoding="utf-8"))


def first_change(trace: list[dict[str, Any]], field: str) -> int:
    return next(i for i in range(1, len(trace)) if trace[i][field] != trace[i - 1][field])


def replay(trace: list[dict[str, Any]]) -> tuple[UrFollowerCore, FakeRTDE, FakeClock]:
    clock = FakeClock()
    rtde = FakeRTDE(trace, clock=clock, hz=HZ)
    return UrFollowerCore(CONFIG, rtde_factory=lambda _ip: rtde, clock=clock), rtde, clock


def follow_until_stopped(core: UrFollowerCore, clock: FakeClock) -> FollowerStoppedError:
    """Lee el estado en cada paquete y manda la pose medida como consigna cada `SEND_EVERY` (lo que
    haría un leader que copia al follower) hasta que el follower para."""
    for step in itertools.count():
        clock.advance_s(1 / HZ)
        try:
            q_rad = core.get_joints()
            if step % SEND_EVERY == 0:
                core.send_joints(q_rad)
        except FollowerStoppedError as exc:
            return exc
    raise AssertionError("inalcanzable")


@pytest.mark.parametrize("name", ["follower_sine", "follower_stopped_from_pendant"])
def test_arms_when_the_real_script_starts_beating(name: str) -> None:
    """En la traza el heartbeat ya late desde el principio: el follower arma en su primer cambio."""
    trace = load_trace(name)
    script_start = first_change(trace, "output_int_register_0")
    core, rtde, _ = replay(trace)
    core.connect()
    assert rtde.received == script_start + 1
    assert trace[script_start]["runtime_state"] == 2  # PLAYING, como se espera tras armar


def test_sine_session_runs_until_the_program_ends() -> None:
    trace = load_trace("follower_sine")
    end = max(i for i, p in enumerate(trace) if p["runtime_state"] == 2) + 1
    core, rtde, clock = replay(trace)
    core.connect()
    stop = follow_until_stopped(core, clock)
    assert "runtime_state=STOPPED" in str(stop)
    assert rtde.received == end + 1  # para en el mismo paquete en que el programa deja de correr
    assert set(rtde.written_enable()[1:]) == {1}


def test_program_stopped_from_pendant_stops_on_that_packet() -> None:
    trace = load_trace("follower_stopped_from_pendant")
    script_start = first_change(trace, "output_int_register_0")
    stopped = next(i for i in range(script_start, len(trace)) if trace[i]["runtime_state"] != 2)
    core, rtde, clock = replay(trace)
    core.connect()
    stop = follow_until_stopped(core, clock)
    assert 1 in rtde.written_enable()
    assert "runtime_state=STOPPED" in str(stop)
    assert rtde.received == stopped + 1
    core.disconnect()
    assert rtde.written_enable()[-1] == 0


def test_leader_times_out_when_the_recorded_timestamp_stops() -> None:
    """Leader quieto grabado en URSim; al acabarse la traza se repite su último paquete, como un
    controlador que siguiera mandando datos con el `timestamp` parado."""
    trace = load_trace("leader_still")
    rtde = FakeRTDE(itertools.chain(trace, itertools.repeat(trace[-1])), hz=HZ)
    leader = UrLeaderCore(CONFIG, rtde_factory=lambda _ip: rtde)
    leader.connect()
    try:
        assert leader.read_joints() == pytest.approx(trace[0]["actual_q"])
        while rtde.received < len(trace):
            leader.read_joints()  # mientras el timestamp avanza, no lanza
            time.sleep(0.01)
        time.sleep(2 * LEADER.timeout_s)
        with pytest.raises(LeaderReadError, match="timestamp"):
            leader.read_joints()
    finally:
        leader.disconnect()
