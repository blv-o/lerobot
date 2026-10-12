"""Los dobles de test no se desvían de las interfaces reales que sustituyen.

Si el cliente RTDE oficial cambia, estos tests fallan antes de que los tests del
follower pasen contra una API que ya no existe.
"""

import inspect
from typing import Any

import pytest
from fakes import START_Q_RAD, FakeClock, FakeRTDE, packet, packets
from rtde.rtde import RTDE, ConnectionState, RTDEException

RTDE_METHODS = [
    "connect",
    "disconnect",
    "is_connected",
    "send_output_setup",
    "send_input_setup",
    "send_start",
    "send",
    "receive",
    "receive_buffered",
]


def _params(func: object) -> list[str]:
    return [name for name in inspect.signature(func).parameters if name != "self"]


@pytest.mark.parametrize("method", RTDE_METHODS)
def test_fake_rtde_methods_match_official_client(method: str) -> None:
    real = _params(getattr(RTDE, method))
    fake = _params(getattr(FakeRTDE, method))
    # receive(binary=False) del cliente real: el follower nunca pide binario.
    if method == "receive":
        real = [p for p in real if p != "binary"]
    assert fake == real


def test_fake_rtde_advances_clock_and_raises_like_real_client_when_exhausted() -> None:
    clock = FakeClock()
    t0_ns = clock()
    con = FakeRTDE(packets(2), clock=clock, hz=125)
    con.connect()
    con.send_start()
    first = con.receive()
    assert first.output_int_register_0 == 0
    assert clock() - t0_ns == 8_000_000
    con.receive()
    with pytest.raises(RTDEException):
        con.receive()


def heartbeats(con: FakeRTDE) -> list[int]:
    """Vacía lo llegado, como el follower, y da el heartbeat de cada paquete."""
    out = []
    while (pkt := con.receive_buffered()) is not None:
        out.append(pkt.output_int_register_0)
    return out


def test_fake_rtde_receive_buffered_delivers_only_what_has_arrived() -> None:
    clock = FakeClock()
    con = FakeRTDE([*packets(2), None, *packets(2)], clock=clock, hz=125)
    con.connect()
    con.send_start()
    assert heartbeats(con) == []
    clock.advance_s(0.020)  # llegan los paquetes de 8 y 16 ms
    assert heartbeats(con) == [0, 1]
    clock.advance_s(0.016)  # a los 24 ms no llega nada (hueco); a los 32 ms, el siguiente
    assert heartbeats(con) == [0]


def test_fake_rtde_receive_buffered_raises_once_when_the_connection_is_lost() -> None:
    """El cliente real lanza al ver la conexión cerrada y desde entonces devuelve None."""
    clock = FakeClock()
    con = FakeRTDE([packet(START_Q_RAD, heartbeat=5)], clock=clock, hz=125)
    con.connect()
    con.send_start()
    clock.advance_s(1.0)
    assert con.receive_buffered().output_int_register_0 == 5
    with pytest.raises(RTDEException):
        con.receive_buffered()
    assert con.receive_buffered() is None
    assert not con.is_connected()


def test_fake_rtde_setup_timeout_raises_like_the_official_client(monkeypatch: pytest.MonkeyPatch) -> None:
    """Sin respuesta a tiempo, el cliente real hace `result.types` sobre None. Si una versión
    nueva lo corrige, este test avisa de que el modo `setup_timeout` ya no la imita."""
    real = RTDE("127.0.0.1")
    monkeypatch.setattr(real, "_RTDE__sendAndReceive", lambda *_args: None)
    fake = FakeRTDE([], fail={"setup_timeout"})
    for con in (real, fake):
        with pytest.raises(AttributeError):
            con.send_output_setup(["actual_q"], ["VECTOR6D"])
        with pytest.raises(AttributeError):
            con.send_input_setup(["input_int_register_0"], ["INT32"])


def test_fake_rtde_inputs_in_use_raises_like_the_official_client(monkeypatch: pytest.MonkeyPatch) -> None:
    """Con un registro ya ocupado el robot responde `IN_USE` y el cliente real lanza ValueError
    al leer la receta, en vez de devolver None."""
    real = RTDE("127.0.0.1")
    monkeypatch.setattr(
        real, "_RTDE__sendAndReceive", lambda cmd, *_args: real._RTDE__on_packet(cmd, b"DOUBLE,IN_USE")
    )
    fake = FakeRTDE([], fail={"inputs_in_use"})
    for con in (real, fake):
        with pytest.raises(ValueError, match="already in use"):
            con.send_input_setup(["input_double_register_0", "input_int_register_0"], ["DOUBLE", "INT32"])


def test_fake_rtde_simulates_setup_failures_as_return_values() -> None:
    con = FakeRTDE([], fail={"send_input_setup", "send_output_setup", "send_start"})
    assert con.send_input_setup(["input_int_register_0"], ["INT32"]) is None
    assert con.send_output_setup(["actual_q"], ["VECTOR6D"]) is False
    assert con.send_start() is False


COMMAND = (["input_double_register_0", "input_int_register_0"], ["DOUBLE", "INT32"])


def _real_with_input_recipe(monkeypatch: pytest.MonkeyPatch) -> tuple[RTDE, Any]:
    """Cliente real con la receta de entrada aceptada, sin red: el robot responde id 1 y los tipos."""
    real = RTDE("127.0.0.1")
    monkeypatch.setattr(
        real, "_RTDE__sendAndReceive", lambda cmd, *_args: real._RTDE__on_packet(cmd, b"DOUBLE,INT32")
    )
    return real, real.send_input_setup(*COMMAND)


def _started_fake_with_input_recipe() -> tuple[FakeRTDE, Any]:
    fake = FakeRTDE([])
    fake.connect()
    fake.send_start()
    return fake, fake.send_input_setup(*COMMAND)


def test_fake_rtde_send_refuses_unset_fields_like_the_official_client(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """El cliente real empaqueta con `serialize.DataObject.pack`, que no admite campos sin valor:
    un bucle que escribiera la receta a medias fallaría con el robot y pasaría con un fake laxo."""
    real, real_command = _real_with_input_recipe(monkeypatch)
    monkeypatch.setattr(real, "_RTDE__conn_state", ConnectionState.STARTED)
    fake, fake_command = _started_fake_with_input_recipe()
    for con, command in ((real, real_command), (fake, fake_command)):
        command.input_int_register_0 = 0
        with pytest.raises(ValueError, match="Uninitialized parameter: input_double_register_0"):
            con.send(command)


def test_fake_rtde_sends_nothing_without_synchronization_like_the_official_client(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Sin sincronización activa (desconectado o conexión perdida) el cliente real no envía y
    devuelve None, sin lanzar."""
    real, real_command = _real_with_input_recipe(monkeypatch)  # nunca conectado
    fake, fake_command = _started_fake_with_input_recipe()
    fake.disconnect()
    for con, command in ((real, real_command), (fake, fake_command)):
        command.input_double_register_0, command.input_int_register_0 = 0.0, 0
        assert con.send(command) is None
    assert fake.sent == []
