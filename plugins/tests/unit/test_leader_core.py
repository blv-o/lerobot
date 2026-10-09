"""`UrLeaderCore`: lectura del leader por RTDE en un hilo, sin escribirle nunca nada.

El hilo corre con `FakeRTDE` a ritmo real (sin reloj falso): los tests esperan a una condición,
nunca un tiempo fijo, salvo para dejar pasar `timeout_ms`.
"""

import dataclasses
import math
import socket
import time
from collections.abc import Iterator
from typing import Any

import pytest
from fakes import START_Q_RAD, FakeRTDE, endless_packets, packet, packets
from ur_core.clock import now_ns
from ur_core.config import (
    UR_TYPES,
    ConfigError,
    FollowerConfig,
    LowCostLeaderConfig,
    ServoConfig,
    TeleopConfig,
    UrLeaderConfig,
    WatchdogConfig,
)
from ur_core.leader import LEADERS, LeaderReadError, UrLeaderCore

LEADER = UrLeaderConfig(type="ur3e", ip="127.0.0.3", rtde_hz=500, timeout_s=0.1)
LEADER_CONFIG = TeleopConfig(
    leader=LEADER,
    follower=FollowerConfig(
        type="ur3e",
        ip="127.0.0.2",
        servo=ServoConfig(hz=125, max_joint_speed_rad_s=math.radians(60), target_period_s=0.033),
        watchdog=WatchdogConfig(hold_s=0.1, stop_s=0.5),
    ),
    start_pose_rad=tuple(START_Q_RAD),
    start_tolerance_rad=math.radians(2),
)
WAIT_S = 5.0


def ticking_packets(q_rad: list[float] = START_Q_RAD, hz: float = 500) -> Iterator[dict[str, Any]]:
    """Leader quieto (misma `actual_q`) con el `timestamp` del controlador avanzando."""
    i = 0
    while True:
        yield packet(q_rad, heartbeat=0, timestamp_s=i / hz)
        i += 1


def moving_packets(hz: float = 500) -> Iterator[dict[str, Any]]:
    """La base gira un poco en cada paquete: cada lectura nueva es distinta de la anterior."""
    i = 0
    while True:
        yield packet([START_Q_RAD[0] + i * 1e-4, *START_Q_RAD[1:]], heartbeat=0, timestamp_s=i / hz)
        i += 1


class Rig:
    """Una `FakeRTDE` nueva por conexión, como el cliente real; guarda todas para los asserts."""

    def __init__(self, pkts: Any = ticking_packets, fail: tuple[str, ...] = ()) -> None:
        self._pkts = pkts
        self._fail = fail
        self.connections: list[FakeRTDE] = []

    def factory(self, _ip: str) -> FakeRTDE:
        self.connections.append(FakeRTDE(self._pkts(), hz=LEADER.rtde_hz, fail=self._fail))
        return self.connections[-1]


@pytest.fixture
def make_leader() -> Iterator[Any]:
    created: list[UrLeaderCore] = []

    def make(rig: Rig, config: Any = LEADER_CONFIG) -> UrLeaderCore:
        core = UrLeaderCore(config, rtde_factory=rig.factory)
        created.append(core)
        return core

    yield make
    for core in created:
        core.disconnect()


def wait_until(predicate: Any, what: str) -> None:
    deadline_ns = now_ns() + round(WAIT_S * 1e9)
    while not predicate():
        assert now_ns() < deadline_ns, f"no se cumplió a tiempo: {what}"
        time.sleep(0.005)


# --- lectura -------------------------------------------------------------------------------


def test_read_joints_right_after_connect_is_the_measured_pose(make_leader: Any) -> None:
    leader = make_leader(Rig())
    leader.connect()
    assert leader.is_connected
    assert leader.read_joints() == pytest.approx(START_Q_RAD)


def test_read_joints_follows_the_latest_actual_q(make_leader: Any) -> None:
    leader = make_leader(Rig(moving_packets))
    leader.connect()
    first = leader.read_joints()
    wait_until(lambda: leader.read_joints()[0] > first[0], "una actual_q más nueva")


def test_reader_thread_is_daemon(make_leader: Any) -> None:
    """`lerobot-teleoperate` conecta el leader fuera de su `try`: si el follower falla al conectar,
    nadie llama a `disconnect()` y un hilo no daemon impediría que el programa terminara."""
    leader = make_leader(Rig())
    leader.connect()
    assert leader._thread is not None and leader._thread.daemon


def test_read_joints_does_not_wait_for_the_robot(make_leader: Any) -> None:
    slow = dataclasses.replace(LEADER, rtde_hz=2, timeout_s=10.0)  # un paquete cada 0,5 s
    leader = make_leader(Rig(), dataclasses.replace(LEADER_CONFIG, leader=slow))
    leader.connect()
    t0_ns = now_ns()
    for _ in range(100):
        leader.read_joints()
    assert (now_ns() - t0_ns) / 1e9 < 0.05


def test_still_leader_with_advancing_timestamp_keeps_reading(make_leader: Any) -> None:
    """En freedrive el leader puede estar quieto: lo que cuenta es el `timestamp`, no `actual_q`."""
    leader = make_leader(Rig(ticking_packets))
    leader.connect()
    time.sleep(3 * LEADER.timeout_s)
    assert leader.read_joints() == pytest.approx(START_Q_RAD)


def test_frozen_timestamp_raises_after_timeout(make_leader: Any) -> None:
    leader = make_leader(Rig(endless_packets))  # timestamp siempre 0
    leader.connect()
    time.sleep(2 * LEADER.timeout_s)
    with pytest.raises(LeaderReadError, match="timestamp"):
        leader.read_joints()


def test_no_data_from_the_robot_raises_after_timeout(make_leader: Any) -> None:
    """El cliente oficial devuelve None si no llega nada en su DEFAULT_TIMEOUT (1 s)."""
    leader = make_leader(Rig(lambda: iter([packet(START_Q_RAD, 0), *[None] * 10_000])))
    leader.connect()
    time.sleep(2 * LEADER.timeout_s)
    with pytest.raises(LeaderReadError):
        leader.read_joints()


def test_lost_connection_raises(make_leader: Any) -> None:
    rig = Rig(lambda: iter(packets(5, hz=LEADER.rtde_hz)))  # al acabarse: RTDEException
    leader = make_leader(rig)
    leader.connect()
    wait_until(lambda: not leader._thread.is_alive(), "el hilo de lectura termina")
    with pytest.raises(LeaderReadError, match="conexión"):
        leader.read_joints()
    assert not rig.connections[0].connected


def test_read_joints_before_connect_raises() -> None:
    with pytest.raises(RuntimeError, match="connect"):
        UrLeaderCore(LEADER_CONFIG).read_joints()


# --- nunca se escribe al leader ------------------------------------------------------------


def test_leader_only_uses_an_output_recipe(make_leader: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    """Ni receta de entrada, ni envíos RTDE, ni ningún socket (30002 o Dashboard) fuera del cliente RTDE."""
    opened: list[Any] = []
    monkeypatch.setattr(socket.socket, "connect", lambda _self, address: opened.append(address))
    monkeypatch.setattr(socket, "create_connection", lambda address, *_a, **_k: opened.append(address))
    rig = Rig()
    leader = make_leader(rig)
    assert leader.check_start() == []
    leader.connect()
    leader.read_joints()
    leader.disconnect()
    assert opened == []
    assert len(rig.connections) == 2  # la breve de check_start y la de lectura
    for con in rig.connections:
        assert con.input_setups == [] and con.sent == []
    assert rig.connections[1].output_setup == (["actual_q", "timestamp"], ["VECTOR6D", "DOUBLE"], 500)


# --- conexión y cierre ---------------------------------------------------------------------


def test_disconnect_stops_the_thread_and_closes_the_connection(make_leader: Any) -> None:
    rig = Rig()
    leader = make_leader(rig)
    leader.connect()
    thread = leader._thread
    leader.disconnect()
    assert not leader.is_connected
    assert not thread.is_alive()
    assert not rig.connections[0].connected


def test_disconnect_twice_or_without_connect_does_nothing(make_leader: Any) -> None:
    leader = make_leader(Rig())
    leader.disconnect()
    leader.connect()
    leader.disconnect()
    leader.disconnect()


def test_connect_twice_does_not_open_a_second_connection(make_leader: Any) -> None:
    rig = Rig()
    leader = make_leader(rig)
    leader.connect()
    leader.connect()
    assert len(rig.connections) == 1


def test_connect_failure_raises_leader_read_error_and_closes_the_socket(make_leader: Any) -> None:
    rig = Rig(fail=("connect_protocol",))
    leader = make_leader(rig)
    with pytest.raises(LeaderReadError, match="RTDE"):
        leader.connect()
    assert not leader.is_connected
    assert not rig.connections[0].connected


def test_connect_failure_when_the_robot_does_not_answer_the_setup(make_leader: Any) -> None:
    with pytest.raises(LeaderReadError, match="no respondió"):
        make_leader(Rig(fail=("setup_timeout",))).connect()


# --- arranque ------------------------------------------------------------------------------


def test_check_start_ok_in_start_pose(make_leader: Any) -> None:
    assert make_leader(Rig()).check_start() == []


def test_check_start_reports_joint_out_of_tolerance(make_leader: Any) -> None:
    q = list(START_Q_RAD)
    q[4] += math.radians(5.3)
    leader = make_leader(Rig(lambda: ticking_packets(q)))
    assert leader.check_start() == ["leader wrist_2: 5,3° fuera de tolerancia"]


def test_check_start_reports_unreachable_rtde(make_leader: Any) -> None:
    errors = make_leader(Rig(fail=("connect",))).check_start()
    assert len(errors) == 1 and errors[0].startswith("leader") and "RTDE" in errors[0]


def test_start_prompt_tells_what_to_prepare_on_the_pendant() -> None:
    prompt = UrLeaderCore(LEADER_CONFIG).start_prompt()
    assert "freedrive" in prompt and "127.0.0.3" in prompt
    assert "[0, -90, 90, -90, -90, 0]" in prompt and "± 2" in prompt
    assert "yes/no" in prompt


def test_low_cost_leader_type_is_rejected() -> None:
    low_cost = LowCostLeaderConfig(
        type="feetech", port="COM3", hz=100, timeout_s=0.1, direction=("normal",) * 6
    )
    with pytest.raises(ConfigError, match="leader.type"):
        UrLeaderCore(dataclasses.replace(LEADER_CONFIG, leader=low_cost))


def test_every_accepted_ur_model_maps_to_the_core() -> None:
    assert set(LEADERS) == set(UR_TYPES)
    assert set(LEADERS.values()) == {UrLeaderCore}
