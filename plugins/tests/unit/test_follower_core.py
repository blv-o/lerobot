"""`UrFollowerCore`: armado, consignas, lectura del estado, paradas y desconexión.

Todo corre en el hilo del test con `FakeRTDE` y `FakeClock`: los paquetes llegan a medida que el
test avanza el reloj, como llegarían al socket entre dos llamadas de LeRobot. Nada duerme.
"""

import logging
import math
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
    leader=UrLeaderConfig(type="ur3e", ip="127.0.0.3", rtde_hz=500, timeout_s=0.1),
    follower=FollowerConfig(type="ur3e", ip="127.0.0.2"),
    start_pose_rad=tuple(START_Q_RAD),
    start_tolerance_rad=math.radians(2),
)
NEAR = [START_Q_RAD[0] + 0.005, *START_Q_RAD[1:]]
FAR = [START_Q_RAD[0] + math.radians(10), *START_Q_RAD[1:]]
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
        return [fields["input_int_register_1"] for _, fields in self.rtde.sent]


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
    first = rig.rtde.sent[0][1]
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
    [({"robot_mode": 5}, "robot_mode"), ({"safety_mode": SAFETY_PROTECTIVE_STOP}, "PROTECTIVE_STOP")],
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
        ("start_lost", "durante la configuración"),
        ("setup_oserror", "durante la configuración"),
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

    def bug() -> Any:
        raise AttributeError("bug nuestro")

    monkeypatch.setattr(rig.core, "_input_setup", bug)
    with pytest.raises(AttributeError, match="bug nuestro"):
        rig.core.connect()
    assert not rig.rtde.connected


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


def test_first_target_far_from_the_measured_pose_stops_without_enabling() -> None:
    rig = armed()
    with pytest.raises(FollowerStoppedError, match="primera consigna lejos de la posición actual"):
        rig.core.send_joints(FAR)
    assert 1 not in rig.rtde.written_enable()
    with pytest.raises(FollowerStoppedError, match="primera consigna lejos"):
        rig.core.send_joints(NEAR)
    with pytest.raises(FollowerStoppedError, match="primera consigna lejos"):
        rig.core.get_joints()
    assert rig.core.is_connected  # hasta disconnect(), aunque el follower ya esté parado


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
    fields = rig.rtde.sent[-1][1]
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
        ({"safety_mode": SAFETY_PROTECTIVE_STOP}, "PROTECTIVE_STOP"),
        ({"robot_mode": 5}, "robot_mode"),
        ({"runtime_state": RUNTIME_STOPPED}, "runtime_state=STOPPED"),
    ],
    ids=["protective_stop", "robot_not_running", "program_stopped_from_pendant"],
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


def test_reduced_mode_is_normal_operation() -> None:
    rig = running(packets(200, safety_mode=SAFETY_REDUCED))
    for _ in range(30):
        rig.tick()
        rig.core.send_joints(NEAR)
        rig.core.get_joints()


@pytest.mark.parametrize("failure", ["raises_oserror", "returns_none"])
def test_a_failed_send_is_a_lost_connection(failure: str) -> None:
    rig = running(packets(30))
    if failure == "raises_oserror":
        rig.rtde._fail.add("send_oserror")  # `sock.sendall` del cliente oficial
    else:
        rig.rtde.started = False  # el cliente oficial registra el error, no envía y devuelve None
    with pytest.raises(FollowerStoppedError, match="se perdió la conexión RTDE"):
        rig.core.send_joints(NEAR)
    with pytest.raises(FollowerStoppedError, match="se perdió la conexión RTDE"):
        rig.core.get_joints()


def test_receive_buffered_raising_is_a_lost_connection() -> None:
    rig = running([])
    rig.tick(1.0)  # se acaban los paquetes: el cliente lanza
    with pytest.raises(FollowerStoppedError, match="se perdió la conexión RTDE"):
        rig.core.get_joints()
    with pytest.raises(FollowerStoppedError, match="se perdió la conexión RTDE"):
        rig.core.send_joints(NEAR)


def test_closed_connection_is_a_lost_connection() -> None:
    rig = running(packets(30))
    rig.rtde.connected = False
    with pytest.raises(FollowerStoppedError, match="se perdió la conexión RTDE"):
        rig.core.get_joints()


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
    last = rig.rtde.sent[-1][1]
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
    assert "enable=0" in caplog.text
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
    assert f"t_ns={stop_ns}" in messages[2] and "PROTECTIVE_STOP" in messages[2]


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
    """Solo el AttributeError del cliente oficial significa "el robot no respondió". Un paquete
    sin el campo pedido sería un fallo de nuestra receta."""

    def rtde_factory(_ip: str) -> FakeRTDE:
        return FakeRTDE([{"timestamp": 0.0}], hz=1000)

    with pytest.raises(AttributeError):
        UrFollowerCore(CONFIG, rtde_factory=rtde_factory).check_start()


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
