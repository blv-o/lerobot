"""`UrFollowerCore`: armado, consignas, lectura del estado, paradas y desconexión.

Todo corre en el hilo del test con `FakeRTDE` y `FakeClock`: los paquetes llegan a medida que el
test avanza el reloj, como llegarían al socket entre dos llamadas de LeRobot. Nada duerme.
"""

import logging
import math
import socket
from collections.abc import Callable, Iterable
from typing import Any

import pytest
import ur_core.follower as follower_module
from fakes import (
    RUNTIME_STOPPED,
    SAFETY_PROTECTIVE_STOP,
    SAFETY_REDUCED,
    START_Q_RAD,
    FakeClock,
    FakeRTDE,
    endless_packets,
    packet,
    packets,
)
from ur_core.config import FollowerConfig, TeleopConfig, UrLeaderConfig
from ur_core.follower import FollowerStartError, FollowerStoppedError, UrFollowerCore

CONFIG = TeleopConfig(
    leader=UrLeaderConfig(type="ur3e", ip="127.0.0.3", timeout_s=0.1),
    follower=FollowerConfig(type="ur3e", ip="127.0.0.2"),
    start_pose_rad=tuple(START_Q_RAD),
    start_tolerance_rad=math.radians(2),
)
NEAR = [START_Q_RAD[0] + 0.005, *START_Q_RAD[1:]]
ARM_PACKETS = 2  # el primero se lee al armar; en el segundo el heartbeat ya ha cambiado
LEROBOT_PERIOD_S = 1 / 30
COMMAND_NAMES = [f"input_double_register_{i}" for i in range(6)] + [
    "input_int_register_0",
    "input_int_register_1",
]


def offset(q_rad: list[float], joint: int, delta_rad: float) -> list[float]:
    out = list(q_rad)
    out[joint] += delta_rad
    return out


class Rig:
    """Follower con su `FakeRTDE` y un reloj falso compartido por los dos."""

    def __init__(
        self, pkts: Iterable[dict[str, Any] | None] | None = None, fail: tuple[str, ...] = ()
    ) -> None:
        self.clock = FakeClock()
        self.rtde = FakeRTDE(endless_packets() if pkts is None else pkts, clock=self.clock, fail=fail)
        self.opened: list[FakeRTDE] = []
        self.core = UrFollowerCore(CONFIG, rtde_factory=self._factory, clock=self.clock)

    def _factory(self, _ip: str) -> FakeRTDE:
        self.opened.append(self.rtde)
        return self.rtde

    def tick(self, dt_s: float = LEROBOT_PERIOD_S) -> None:
        self.clock.advance_s(dt_s)

    def counters(self) -> list[int]:
        return [fields["input_int_register_1"] for fields in self.rtde.sent]


def armed(pkts: Iterable[dict[str, Any] | None] | None = None) -> Rig:
    rig = Rig(pkts)
    rig.core.connect()
    return rig


def running(tail: list[dict[str, Any] | None]) -> Rig:
    """Armado y habilitado con una consigna cercana; luego llegan los paquetes de `tail`."""
    rig = armed([*packets(ARM_PACKETS + 3), *tail])
    rig.core.send_joints(NEAR)
    return rig


# --- armado --------------------------------------------------------------------------------


def test_connect_declares_typed_recipes_at_125_hz() -> None:
    rig = armed()
    names, types, frequency_hz = rig.rtde.output_setup
    assert names == [
        "actual_q",
        "timestamp",
        "robot_mode",
        "safety_mode",
        "runtime_state",
        "output_int_register_0",
    ]
    assert types == ["VECTOR6D", "DOUBLE", "INT32", "INT32", "UINT32", "INT32"]
    assert frequency_hz == 125
    # Una sola receta de entrada: consigna, enable y contador llegan siempre juntos al robot.
    assert rig.rtde.input_setups == [(COMMAND_NAMES, ["DOUBLE"] * 6 + ["INT32", "INT32"])]


def test_connect_first_write_is_the_measured_pose_disabled() -> None:
    measured = offset(START_Q_RAD, 3, 0.01)
    rig = armed(packets(5, q_rad=measured))
    first = rig.rtde.sent[0]
    assert [first[f"input_double_register_{i}"] for i in range(6)] == measured
    assert first["input_int_register_0"] == 0


def test_every_write_changes_the_counter() -> None:
    """El programa del TP espera a que el contador cambie (no a un valor: los registros guardan
    el último tras desconectar) y cada cambio es una consigna nueva."""
    rig = armed()
    for _ in range(3):
        rig.core.send_joints(NEAR)
        rig.tick()
    rig.core.disconnect()
    counters = rig.counters()
    assert len(counters) >= 5
    assert all(a != b for a, b in zip(counters, counters[1:], strict=False))


def test_connect_rewrites_enable_0_with_a_new_counter_until_the_program_beats() -> None:
    """El programa del TP se arma al ver un contador nuevo con enable=0, y solo late ya armado. Un
    contador nuevo en cada paquete lo arma aunque el registro ya tuviera el valor de la primera
    escritura (p. ej. de un arranque anterior que falló)."""
    still = [packet(START_Q_RAD, heartbeat=7) for _ in range(10)]
    rig = armed([*still, *packets(5)])
    assert len(rig.rtde.sent) >= len(still)
    assert set(rig.rtde.written_enable()) == {0}
    counters = rig.counters()
    assert len(set(counters)) == len(counters)


def test_connect_returns_as_soon_as_the_heartbeat_changes() -> None:
    still = [packet(START_Q_RAD, heartbeat=7) for _ in range(10)]
    rig = armed([*still, *packets(5)])
    assert rig.core.is_connected
    assert rig.rtde.received == len(still) + 1


def test_connect_without_heartbeat_asks_to_press_play() -> None:
    arm_timeout_s = follower_module.ARM_TIMEOUT_S
    frozen = [packet(START_Q_RAD, heartbeat=7) for _ in range(round(arm_timeout_s * 125) + 10)]
    rig = Rig(frozen)
    t0_ns = rig.clock()
    with pytest.raises(
        FollowerStartError, match="el programa del follower no está en marcha: dale a Play en el TP"
    ):
        rig.core.connect()
    assert rig.clock() - t0_ns >= arm_timeout_s * 1e9
    assert rig.rtde.received < len(frozen)
    assert not rig.core.is_connected and not rig.rtde.connected
    assert 1 not in rig.rtde.written_enable()


@pytest.mark.parametrize(
    ("modes", "fragment"),
    [({"robot_mode": 5}, "robot_mode"), ({"safety_mode": SAFETY_PROTECTIVE_STOP}, "safety_mode=3")],
    ids=["not_running", "protective_stop"],
)
def test_connect_refuses_a_robot_that_is_not_ready(modes: dict[str, int], fragment: str) -> None:
    rig = Rig(packets(5, **modes))
    with pytest.raises(FollowerStartError, match=fragment):
        rig.core.connect()
    assert rig.rtde.sent == []
    assert not rig.core.is_connected and not rig.rtde.connected


@pytest.mark.parametrize(
    ("fail", "fragment"),
    [
        ("connect", "conectar"),
        ("connect_protocol", "arrancando"),
        ("setup_timeout", "no respond"),
        ("send_output_setup", "salida"),
        ("send_input_setup", "entrada"),
        ("inputs_in_use", "bus de campo"),
        ("send_start", "iniciar"),
        ("start_lost", "conectar"),
        ("setup_oserror", "conectar"),
    ],
)
def test_rtde_setup_failures_give_a_readable_start_error_and_close_the_socket(
    fail: str, fragment: str
) -> None:
    rig = Rig(fail=(fail,))
    with pytest.raises(FollowerStartError, match=fragment):
        rig.core.connect()
    assert not rig.core.is_connected
    assert not rig.rtde.connected  # el cliente oficial deja el socket abierto si falla la negociación


def test_a_failure_to_close_does_not_hide_the_start_error(caplog: pytest.LogCaptureFixture) -> None:
    """`sock.close` del cliente oficial puede lanzar OSError: el operador tiene que ver por qué no
    arrancó, no el fallo al cerrar."""
    rig = Rig(packets(5, robot_mode=5), fail=("disconnect",))
    with pytest.raises(FollowerStartError, match="robot_mode"):
        rig.core.connect()
    assert not rig.core.is_connected
    assert "fallo al cerrar el socket" in caplog.text


def test_our_own_attribute_error_during_setup_is_not_blamed_on_the_robot(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Solo el AttributeError del cliente oficial (respuesta que no llegó) significa "el robot no
    respondió"; uno de nuestro código tiene que verse como lo que es."""
    rig = Rig()

    def bug(*_args: Any, **_kwargs: Any) -> Any:
        raise AttributeError("bug nuestro")

    monkeypatch.setattr(follower_module, "start_output", bug)
    with pytest.raises(AttributeError, match="bug nuestro"):
        rig.core.connect()
    assert not rig.rtde.connected


@pytest.mark.parametrize(
    ("pkts", "fragment"),
    [([None], "sin datos"), ([], "se perdió la conexión")],
    ids=["receive_returns_none", "receive_raises"],
)
def test_no_data_while_arming_is_a_start_error(pkts: list[dict[str, Any] | None], fragment: str) -> None:
    """`receive()` del cliente oficial devuelve None tras su timeout y lanza al perder la conexión."""
    rig = Rig(pkts)
    with pytest.raises(FollowerStartError, match=fragment):
        rig.core.connect()
    assert not rig.core.is_connected and not rig.rtde.connected


@pytest.mark.parametrize(
    "call",
    [lambda core: core.get_joints(), lambda core: core.send_joints(NEAR)],
    ids=["get_joints", "send_joints"],
)
def test_relaunch_with_the_program_already_enabled_stops_on_the_first_call(call: Callable[..., Any]) -> None:
    """Sin Stop y Play tras un PC colgado, el heartbeat late y `connect()` pasa. Si el programa
    deja de estar en Play tras el armado, la primera llamada lo ve (el fake no simula el script)."""
    rig = armed([*packets(ARM_PACKETS), *packets(30, runtime_state=RUNTIME_STOPPED)])
    rig.tick()
    with pytest.raises(FollowerStoppedError, match="runtime_state=1"):
        call(rig.core)
    assert 1 not in rig.rtde.written_enable()


def test_connect_twice_opens_a_single_connection() -> None:
    rig = armed()
    rig.core.connect()
    assert rig.opened == [rig.rtde]


# --- consignas -----------------------------------------------------------------------------


def test_without_targets_the_follower_is_never_enabled() -> None:
    rig = armed()
    for _ in range(60):
        rig.tick()
        assert rig.core.get_joints() == pytest.approx(START_Q_RAD)
    rig.core.disconnect()
    assert set(rig.rtde.written_enable()) == {0}


@pytest.mark.parametrize(("offset_deg", "enables"), [(3.0, True), (3.9, True), (4.1, False)])
def test_first_target_may_be_up_to_twice_the_start_tolerance_away(offset_deg: float, enables: bool) -> None:
    """Cada robot puede arrancar a ± tolerancia de la posición inicial: entre sí, hasta el doble."""
    rig = armed()
    first = offset(START_Q_RAD, 2, math.radians(offset_deg))
    if enables:
        rig.core.send_joints(first)
    else:
        with pytest.raises(FollowerStoppedError, match="primera consigna lejos"):
            rig.core.send_joints(first)
    assert (1 in rig.rtde.written_enable()) == enables


def test_send_joints_writes_target_enable_and_counter_in_one_packet_without_receiving(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    rig = armed()
    monkeypatch.setattr(rig.rtde, "receive", lambda: pytest.fail("send_joints no debe esperar al robot"))
    before = len(rig.rtde.sent)
    rig.core.send_joints(NEAR)
    assert len(rig.rtde.sent) == before + 1
    fields = rig.rtde.sent[-1]
    assert [fields[f"input_double_register_{i}"] for i in range(6)] == NEAR
    assert fields["input_int_register_0"] == 1
    assert rig.counters()[-1] != rig.counters()[-2]


@pytest.mark.parametrize(
    "bad",
    [
        START_Q_RAD[:5],
        [*START_Q_RAD, 0.0],
        [math.nan, *START_Q_RAD[1:]],
        [math.inf, *START_Q_RAD[1:]],
    ],
)
def test_send_joints_rejects_wrong_size_and_non_finite(bad: list[float]) -> None:
    rig = armed()
    with pytest.raises(ValueError):
        rig.core.send_joints(bad)
    assert 1 not in rig.rtde.written_enable()  # nunca llegó al robot
    rig.core.send_joints(NEAR)  # y no es una parada


def test_send_joints_before_connect_raises() -> None:
    with pytest.raises(RuntimeError, match="no conectado"):
        Rig().core.send_joints(NEAR)


# --- lectura del estado --------------------------------------------------------------------


def test_get_joints_drains_the_buffer_and_returns_the_newest_packet() -> None:
    moving = [packet(offset(START_Q_RAD, 0, k * 1e-3), heartbeat=k) for k in range(40)]
    rig = armed(moving)
    rig.tick(0.050)  # llegan los paquetes 2..7
    assert rig.core.get_joints() == pytest.approx(moving[7]["actual_q"])
    assert rig.rtde.received == 8
    assert rig.core.get_joints() == pytest.approx(moving[7]["actual_q"])  # sin paquetes nuevos


def test_receive_is_never_called_after_connect(monkeypatch: pytest.MonkeyPatch) -> None:
    """`receive()` del cliente oficial bloquea hasta el siguiente paquete: el bucle de LeRobot no
    debe esperar al robot."""
    rig = armed()
    monkeypatch.setattr(rig.rtde, "receive", lambda: pytest.fail("receive() tras connect()"))
    for _ in range(5):
        rig.core.send_joints(NEAR)
        rig.tick()
        rig.core.get_joints()
    rig.core.disconnect()


Call = Callable[[UrFollowerCore], Any]
CALLS: list[Call] = [lambda core: core.get_joints(), lambda core: core.send_joints(NEAR)]


@pytest.mark.parametrize("call", CALLS, ids=["get_joints", "send_joints"])
@pytest.mark.parametrize(
    ("modes", "fragment"),
    [
        ({"safety_mode": SAFETY_PROTECTIVE_STOP}, "safety_mode=3"),
        ({"robot_mode": 5}, "robot_mode"),
        ({"runtime_state": RUNTIME_STOPPED}, "runtime_state=1"),
        ({"safety_mode": 6}, "safety_mode=6"),
        ({"safety_mode": 7}, "safety_mode=7"),
    ],
    ids=[
        "protective_stop",
        "robot_not_running",
        "program_stopped_from_pendant",
        "system_emergency_stop",
        "robot_emergency_stop",
    ],
)
def test_robot_stops_raise_with_reason_and_so_do_all_later_calls(
    modes: dict[str, int], fragment: str, call: Call
) -> None:
    rig = running(packets(30, **modes))
    rig.tick(0.1)
    with pytest.raises(FollowerStoppedError, match=fragment):
        call(rig.core)
    for later in CALLS:
        with pytest.raises(FollowerStoppedError, match=fragment):
            later(rig.core)
    assert rig.core.is_connected


@pytest.mark.parametrize("call", CALLS, ids=["get_joints", "send_joints"])
def test_a_stop_hidden_behind_newer_packets_still_stops(call: Call) -> None:
    """Stop y Play en el TP mientras LeRobot no lee (p. ej. `save_episode`): el paquete más nuevo
    vuelve a decir PLAYING, pero el programa nuevo ya no es el que estaba siguiendo las consignas."""
    rig = running([*packets(3), *packets(2, runtime_state=RUNTIME_STOPPED), *packets(30)])
    rig.tick(0.1)
    with pytest.raises(FollowerStoppedError, match="runtime_state"):
        call(rig.core)


def test_reduced_mode_is_normal_operation() -> None:
    rig = armed(packets(200, safety_mode=SAFETY_REDUCED))
    assert rig.core.is_connected
    for _ in range(30):
        rig.tick()
        rig.core.send_joints(NEAR)
        rig.core.get_joints()


def send_raises(rig: Rig) -> None:
    rig.rtde._fail.add("send_oserror")  # `sock.sendall` del cliente oficial


def send_returns_none(rig: Rig) -> None:
    rig.rtde.started = False  # el cliente oficial registra el error, no envía y devuelve None


def receive_buffered_raises(rig: Rig) -> None:
    rig.tick(1.0)  # se acaban los paquetes: el cliente lanza


def connection_closed(rig: Rig) -> None:
    rig.rtde.connected = False


@pytest.mark.parametrize(
    ("cause", "calls"),
    [
        (send_raises, CALLS[::-1]),  # solo `send_joints` envía: es la que lo ve
        (send_returns_none, CALLS[::-1]),
        (receive_buffered_raises, CALLS),
        (connection_closed, CALLS),
    ],
    ids=["send_raises", "send_returns_none", "receive_buffered_raises", "connection_closed"],
)
def test_causes_of_a_lost_connection(cause: Callable[[Rig], None], calls: list[Call]) -> None:
    rig = running(packets(30))
    cause(rig)
    for call in calls:  # la primera lo detecta; la siguiente lanza igual
        with pytest.raises(FollowerStoppedError, match="se perdió la conexión RTDE"):
            call(rig.core)


def test_no_new_packet_for_stale_time_is_a_lost_connection() -> None:
    """Cable caído sin cierre: la conexión sigue abierta pero no llega nada."""
    stale_s = follower_module.STALE_S
    rig = armed([*packets(ARM_PACKETS), *[None] * 300])  # el último paquete llega al armar
    rig.core.send_joints(NEAR)
    rig.tick(0.9 * stale_s)
    rig.core.get_joints()
    rig.tick(0.2 * stale_s)
    with pytest.raises(FollowerStoppedError, match="ningún paquete nuevo"):
        rig.core.get_joints()


def test_a_long_pause_drains_the_backlog_without_a_false_stale_stop() -> None:
    """P. ej. `save_episode` entre episodios: los paquetes se acumulan y llegan todos de golpe."""
    rig = armed()
    rig.core.send_joints(NEAR)
    rig.tick(30.0)
    assert rig.core.get_joints() == pytest.approx(START_Q_RAD)
    rig.core.send_joints(NEAR)


# --- desconexión ---------------------------------------------------------------------------


def test_disconnect_writes_the_last_target_disabled_and_closes() -> None:
    rig = armed()
    rig.core.send_joints(NEAR)
    rig.core.disconnect()
    last = rig.rtde.sent[-1]
    assert [last[f"input_double_register_{i}"] for i in range(6)] == NEAR
    assert last["input_int_register_0"] == 0
    assert not rig.rtde.connected and not rig.core.is_connected


def test_disconnect_twice_or_without_connect_does_nothing() -> None:
    rig = Rig()
    rig.core.disconnect()
    assert rig.opened == []
    rig.core.connect()
    rig.core.disconnect()
    sent = len(rig.rtde.sent)
    rig.core.disconnect()
    assert len(rig.rtde.sent) == sent


def test_disconnect_after_losing_the_connection_only_closes() -> None:
    rig = armed()
    rig.core.send_joints(NEAR)
    rig.rtde.connected = False
    sent = len(rig.rtde.sent)
    rig.core.disconnect()
    assert len(rig.rtde.sent) == sent
    assert not rig.core.is_connected


def test_disconnect_with_a_broken_socket_logs_it_and_still_closes(caplog: pytest.LogCaptureFixture) -> None:
    """En el `finally` de LeRobot: lanzar aquí impediría cerrar el leader y las cámaras."""
    rig = armed()
    rig.core.send_joints(NEAR)
    rig.rtde._fail.add("send_oserror")
    rig.core.disconnect()
    assert any("enable=0" in r.getMessage() and "t_ns=" in r.getMessage() for r in caplog.records)
    assert not rig.rtde.connected and not rig.core.is_connected


def test_disconnect_with_a_socket_that_fails_to_close_logs_it_and_does_not_raise(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """`sock.close` del cliente oficial puede lanzar OSError; en el `finally` de LeRobot eso
    impediría cerrar el leader y las cámaras."""
    rig = armed()
    rig.rtde._fail.add("disconnect")
    rig.core.disconnect()
    assert not rig.core.is_connected
    assert "fallo al cerrar el socket" in caplog.text
    assert any("cerrar" in r.getMessage() and "t_ns=" in r.getMessage() for r in caplog.records)


def test_follower_opens_no_socket_besides_the_rtde_client(monkeypatch: pytest.MonkeyPatch) -> None:
    """Ni script por 30002 ni comandos al Dashboard (29999): el programa del TP lo gobierna todo."""
    opened: list[Any] = []
    monkeypatch.setattr(socket.socket, "connect", lambda _self, address: opened.append(address))
    monkeypatch.setattr(socket, "create_connection", lambda address, *_a, **_k: opened.append(address))
    clock = FakeClock()
    connections: list[FakeRTDE] = []

    def rtde_factory(_ip: str) -> FakeRTDE:
        connections.append(FakeRTDE(endless_packets(), clock=clock))
        return connections[-1]

    core = UrFollowerCore(CONFIG, rtde_factory=rtde_factory, clock=clock)
    assert core.check_start() == []
    core.connect()
    core.send_joints(NEAR)
    clock.advance_s(LEROBOT_PERIOD_S)
    core.get_joints()
    core.disconnect()
    assert opened == []
    assert len(connections) == 2  # la breve de check_start y la de control


# --- observabilidad ------------------------------------------------------------------------


def test_arming_enabling_and_stopping_are_logged_with_t_ns(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO, logger="ur_core.follower"):
        rig = running(packets(30, safety_mode=SAFETY_PROTECTIVE_STOP))
        rig.tick(0.1)
        stop_ns = rig.clock()
        with pytest.raises(FollowerStoppedError):
            rig.core.get_joints()
    messages = [record.getMessage() for record in caplog.records if "t_ns=" in record.getMessage()]
    assert len(messages) == 3
    assert "armado" in messages[0]
    assert "habilitado" in messages[1]
    assert f"t_ns={stop_ns}" in messages[2] and "safety_mode=3" in messages[2]


# --- comprobaciones de arranque --------------------------------------------------------------


def make_core(q_rad: list[float] = START_Q_RAD, rtde_fail: tuple[str, ...] = ()) -> UrFollowerCore:
    def rtde_factory(_ip: str) -> FakeRTDE:
        return FakeRTDE(packets(1, q_rad=q_rad), hz=1000, fail=rtde_fail)

    return UrFollowerCore(CONFIG, rtde_factory=rtde_factory)


def test_check_start_reads_the_follower_pose() -> None:
    """La comparación con la posición inicial la prueba `test_startup.py`; aquí, que lee el follower."""
    q = offset(START_Q_RAD, 4, math.radians(5.3))
    assert make_core(q_rad=q).check_start() == ["follower wrist_2: 5,3° fuera de tolerancia"]


def test_check_start_reports_robot_not_answering_the_rtde_setup() -> None:
    """El cliente oficial lanza AttributeError si la respuesta a la receta no llega a tiempo."""
    errors = make_core(rtde_fail=("setup_timeout",)).check_start()
    assert len(errors) == 1
    assert "RTDE" in errors[0] and "no respondió" in errors[0]


def test_start_prompt_tells_what_to_prepare_on_the_pendant() -> None:
    prompt = UrFollowerCore(CONFIG).start_prompt()
    assert "Play" in prompt and "Remote" not in prompt
    assert "[0, -90, 90, -90, -90, 0]" in prompt and "± 2" in prompt
    assert "yes/no" in prompt
    assert prompt.endswith("\n")  # con `| Tee-Object`, sin él la pregunta no se ve
