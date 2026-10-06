"""Los dobles de test no se desvían de las interfaces reales que sustituyen.

Si el cliente RTDE oficial o nuestros clientes de Dashboard/secundaria cambian, estos tests
fallan antes de que los tests del follower pasen contra una API que ya no existe.
"""

import inspect

import pytest
from fakes import FakeClock, FakeDashboard, FakeRTDE, FakeSecondary, packets
from rtde.rtde import RTDE, RTDEException
from ur_core.dashboard import DashboardClient
from ur_core.secondary import send_script

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


def test_fake_secondary_matches_send_script_required_params() -> None:
    required = [
        name
        for name, p in inspect.signature(send_script).parameters.items()
        if p.default is inspect.Parameter.empty
    ]
    assert _params(FakeSecondary.__call__) == required


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
