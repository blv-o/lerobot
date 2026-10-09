"""Los dobles de test no se desvían de las interfaces reales que sustituyen.

Si el cliente RTDE oficial o nuestro cliente de Dashboard cambian, estos tests
fallan antes de que los tests del follower pasen contra una API que ya no existe.
"""

import inspect
from typing import Any

import pytest
from fakes import FakeClock, FakeDashboard, FakeRTDE, packets
from rtde.rtde import RTDE, ConnectionState, RTDEException
from ur_core.dashboard import DashboardClient

RTDE_METHODS = [
    "connect",
    "disconnect",
    "is_connected",
    "send_output_setup",
    "send_input_setup",
    "send_start",
    "send_pause",
    "send",
    "receive",
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


@pytest.mark.parametrize("method", ["connect", "send", "close"])
def test_fake_dashboard_methods_match_client(method: str) -> None:
    assert _params(getattr(FakeDashboard, method)) == _params(getattr(DashboardClient, method))


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


def test_fake_rtde_runs_actions_before_packet_index() -> None:
    seen: list[int] = []
    con = FakeRTDE(packets(3), clock=FakeClock(), actions={1: lambda: seen.append(con.received)})
    con.connect()
    con.send_start()
    con.receive()
    assert seen == []
    con.receive()
    assert seen == [1]


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
