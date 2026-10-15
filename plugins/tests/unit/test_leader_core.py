"""`UrLeaderCore`: lectura del leader por RTDE, sin escribirle nunca nada.

Todo corre en el hilo del test con `FakeRTDE` y `FakeClock`: los paquetes llegan a medida que el
test avanza el reloj, como llegarían al socket entre dos llamadas de LeRobot. Nada duerme.
"""

import dataclasses
import logging
import math
import socket
from collections.abc import Iterator
from typing import Any

import pytest
from fakes import START_Q_RAD, FakeClock, FakeRTDE, endless_packets, packet, packets, ticking_packets
from ur_core.config import ConfigError, FollowerConfig, LowCostLeaderConfig, TeleopConfig, UrLeaderConfig
from ur_core.leader import LeaderReadError, UrLeaderCore

LEADER = UrLeaderConfig(type="ur3e", ip="127.0.0.3", timeout_s=0.1)
LEADER_CONFIG = TeleopConfig(
    leader=LEADER,
    follower=FollowerConfig(type="ur3e", ip="127.0.0.2"),
    start_pose_rad=tuple(START_Q_RAD),
    start_tolerance_rad=math.radians(2),
)
LEROBOT_PERIOD_S = 1 / 30


def moving_packets() -> Iterator[dict[str, Any]]:
    """La base gira un poco en cada paquete: cada lectura nueva es distinta de la anterior."""
    i = 0
    while True:
        yield packet([START_Q_RAD[0] + i * 1e-4, *START_Q_RAD[1:]], heartbeat=0, timestamp_s=i / 125)
        i += 1


class Rig:
    """Una `FakeRTDE` nueva por conexión, como el cliente real, con un reloj falso compartido."""

    def __init__(self, pkts: Any = ticking_packets, fail: tuple[str, ...] = ()) -> None:
        self._pkts = pkts
        self._fail = fail
        self.clock = FakeClock()
        self.connections: list[FakeRTDE] = []
        self.leader = UrLeaderCore(LEADER_CONFIG, rtde_factory=self.factory, clock=self.clock)

    def factory(self, _ip: str) -> FakeRTDE:
        self.connections.append(FakeRTDE(self._pkts(), clock=self.clock, fail=self._fail))
        return self.connections[-1]

    def tick(self, dt_s: float = LEROBOT_PERIOD_S) -> None:
        self.clock.advance_s(dt_s)


def connected(pkts: Any = ticking_packets) -> Rig:
    rig = Rig(pkts)
    rig.leader.connect()
    return rig


# --- lectura -------------------------------------------------------------------------------


def test_read_joints_right_after_connect_is_the_measured_pose() -> None:
    rig = connected()
    assert rig.leader.is_connected
    assert rig.leader.read_joints() == pytest.approx(START_Q_RAD)


def test_read_joints_drains_the_buffer_and_returns_the_newest_packet() -> None:
    rig = connected(moving_packets)
    first = rig.leader.read_joints()
    rig.tick(0.050)
    assert rig.leader.read_joints()[0] > first[0]
    assert rig.connections[0].received == 1 + 6  # el de connect() y los 6 llegados en 50 ms


def test_receive_is_never_called_after_connect(monkeypatch: pytest.MonkeyPatch) -> None:
    """`receive()` del cliente oficial bloquea hasta el siguiente paquete: el bucle de LeRobot no
    debe esperar al robot."""
    rig = connected()
    monkeypatch.setattr(rig.connections[0], "receive", lambda: pytest.fail("receive() tras connect()"))
    for _ in range(5):
        rig.tick()
        rig.leader.read_joints()


def test_the_backlog_is_drained_in_bounded_chunks() -> None:
    """P. ej. el leader sin leer mientras se contesta la pregunta del follower: sin límite, el
    cliente oficial copia todo su buffer en cada paquete y el primer tick se congela."""
    rig = connected()
    rig.tick(30.0)
    rig.leader.read_joints()
    limits = rig.connections[0].buffer_limits
    assert limits and all(isinstance(limit, int) and limit > 0 for limit in limits)


def test_still_leader_with_advancing_timestamp_keeps_reading() -> None:
    """En freedrive el leader puede estar quieto: lo que cuenta es el `timestamp`, no `actual_q`."""
    rig = connected(ticking_packets)
    for _ in range(10):
        rig.tick(LEADER.timeout_s / 2)
        assert rig.leader.read_joints() == pytest.approx(START_Q_RAD)


def test_frozen_timestamp_raises_after_timeout() -> None:
    rig = connected(endless_packets)  # timestamp siempre 0
    rig.tick(0.9 * LEADER.timeout_s)
    rig.leader.read_joints()
    rig.tick(0.2 * LEADER.timeout_s)
    with pytest.raises(LeaderReadError, match="timestamp"):
        rig.leader.read_joints()


def test_no_data_from_the_robot_raises_after_timeout() -> None:
    rig = connected(lambda: iter([packet(START_Q_RAD, 0), *[None] * 10_000]))
    rig.tick(2 * LEADER.timeout_s)
    with pytest.raises(LeaderReadError):
        rig.leader.read_joints()


def test_lost_connection_raises_and_is_logged_with_t_ns(caplog: pytest.LogCaptureFixture) -> None:
    rig = connected(lambda: iter(packets(5)))  # al acabarse: RTDEException
    rig.tick(1.0)
    for _ in range(2):  # también las llamadas siguientes
        with pytest.raises(LeaderReadError, match="se perdió la conexión"):
            rig.leader.read_joints()
    assert rig.leader.is_connected  # hasta disconnect(), aunque se perdiera la conexión
    assert any(
        "se perdió la conexión" in r.getMessage() and "t_ns=" in r.getMessage() for r in caplog.records
    )


@pytest.mark.parametrize(
    "pkts", [endless_packets, lambda: iter(packets(5))], ids=["frozen_timestamp", "lost_connection"]
)
def test_every_read_error_tells_the_operator_what_to_do(pkts: Any) -> None:
    rig = connected(pkts)
    rig.tick(1.0)
    with pytest.raises(LeaderReadError, match="comprueba el programa de freedrive y el cable del leader$"):
        rig.leader.read_joints()


def test_a_failed_connect_tells_what_to_do_and_is_logged_with_t_ns(caplog: pytest.LogCaptureFixture) -> None:
    with pytest.raises(LeaderReadError, match="comprueba el programa de freedrive y el cable del leader$"):
        Rig(fail=("connect",)).leader.connect()
    assert any("t_ns=" in r.getMessage() for r in caplog.records)


def test_read_joints_before_connect_raises() -> None:
    with pytest.raises(RuntimeError, match="connect"):
        UrLeaderCore(LEADER_CONFIG).read_joints()


# --- nunca se escribe al leader ------------------------------------------------------------


def test_leader_only_uses_an_output_recipe(monkeypatch: pytest.MonkeyPatch) -> None:
    """Ni receta de entrada, ni envíos RTDE, ni ningún socket (30002 o Dashboard) fuera del cliente RTDE."""
    opened: list[Any] = []
    monkeypatch.setattr(socket.socket, "connect", lambda _self, address: opened.append(address))
    monkeypatch.setattr(socket, "create_connection", lambda address, *_a, **_k: opened.append(address))
    rig = Rig()
    assert rig.leader.check_start() == []
    rig.leader.connect()
    rig.tick()
    rig.leader.read_joints()
    rig.leader.disconnect()
    assert opened == []
    assert len(rig.connections) == 2  # la breve de check_start y la de lectura
    for con in rig.connections:
        assert con.input_setups == [] and con.sent == []
    assert rig.connections[1].output_setup == (["actual_q", "timestamp"], ["VECTOR6D", "DOUBLE"], 125)


# --- conexión y cierre ---------------------------------------------------------------------


def test_disconnect_closes_the_connection() -> None:
    rig = connected()
    rig.leader.disconnect()
    assert not rig.leader.is_connected
    assert not rig.connections[0].connected


def test_disconnect_twice_or_without_connect_does_nothing() -> None:
    rig = Rig()
    rig.leader.disconnect()
    rig.leader.connect()
    rig.leader.disconnect()
    rig.leader.disconnect()


def test_connect_twice_does_not_open_a_second_connection() -> None:
    rig = connected()
    rig.leader.connect()
    assert len(rig.connections) == 1


def test_connect_failure_raises_leader_read_error_and_closes_the_socket() -> None:
    rig = Rig(fail=("connect_protocol",))
    with pytest.raises(LeaderReadError, match="RTDE"):
        rig.leader.connect()
    assert not rig.leader.is_connected
    assert not rig.connections[0].connected


def test_connect_failure_when_the_robot_does_not_answer_the_setup() -> None:
    with pytest.raises(LeaderReadError, match="no respondió"):
        Rig(fail=("setup_timeout",)).leader.connect()


# --- arranque ------------------------------------------------------------------------------


def test_check_start_reads_the_leader_pose() -> None:
    """La comparación con la posición inicial la prueba `test_startup.py`; aquí, que lee el leader."""
    q = list(START_Q_RAD)
    q[4] += math.radians(5.3)
    rig = Rig(lambda: ticking_packets(q))
    assert rig.leader.check_start() == ["leader wrist_2: 5,3° fuera de tolerancia"]


def test_start_prompt_tells_what_to_prepare_on_the_pendant() -> None:
    prompt = UrLeaderCore(LEADER_CONFIG).start_prompt()
    assert "freedrive" in prompt and "127.0.0.3" in prompt
    assert "[0, -90, 90, -90, -90, 0]" in prompt and "± 2" in prompt
    assert "yes/no" in prompt
    assert prompt.endswith("\n")  # con `| Tee-Object`, sin él la pregunta no se ve


def test_low_cost_leader_type_is_rejected() -> None:
    low_cost = LowCostLeaderConfig(
        type="feetech", port="COM3", hz=100, timeout_s=0.1, direction=("normal",) * 6
    )
    with pytest.raises(ConfigError, match="leader.type"):
        UrLeaderCore(dataclasses.replace(LEADER_CONFIG, leader=low_cost))


def test_connect_and_disconnect_are_logged_with_t_ns(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO, logger="ur_core.leader"):
        rig = connected()
        rig.leader.disconnect()
    messages = [r.getMessage() for r in caplog.records if "t_ns=" in r.getMessage()]
    assert any("conectado" in m for m in messages)
    assert any("cerrado" in m for m in messages)
