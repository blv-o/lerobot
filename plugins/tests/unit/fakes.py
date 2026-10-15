"""Dobles de test de los robots UR: RTDE y reloj.

Los paquetes RTDE son dicts con los campos de la receta de salida; así la misma `FakeRTDE`
sirve para paquetes escritos a mano y para trazas grabadas en URSim.
"""

import math
import time
from collections.abc import Iterable, Iterator
from types import SimpleNamespace
from typing import Any

from rtde.rtde import RTDEException
from ur_core.clock import now_ns

ROBOT_RUNNING = 7
SAFETY_NORMAL = 1
SAFETY_REDUCED = 2
SAFETY_PROTECTIVE_STOP = 3
RUNTIME_PLAYING = 2
RUNTIME_STOPPED = 1

START_Q_RAD = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
ROBOT_HZ = 125  # el robot manda su estado cada 8 ms


class FakeClock:
    """Reloj en ns que solo avanza cuando el test (o `FakeRTDE`) lo pide: nada duerme."""

    def __init__(self) -> None:
        self.now_ns = 1_000_000_000

    def __call__(self) -> int:
        return self.now_ns

    def advance_s(self, dt_s: float) -> None:
        self.now_ns += round(dt_s * 1e9)


def packet(
    q_rad: list[float],
    heartbeat: int,
    timestamp_s: float = 0.0,
    robot_mode: int = ROBOT_RUNNING,
    safety_mode: int = SAFETY_NORMAL,
    runtime_state: int = RUNTIME_PLAYING,
) -> dict[str, Any]:
    return {
        "actual_q": list(q_rad),
        "timestamp": timestamp_s,
        "robot_mode": robot_mode,
        "safety_mode": safety_mode,
        "runtime_state": runtime_state,
        "output_int_register_0": heartbeat,
    }


def packets(n: int, q_rad: list[float] = START_Q_RAD, **modes: int) -> list[dict[str, Any]]:
    """`n` paquetes de un robot quieto en `q_rad` con el heartbeat latiendo."""
    return [packet(q_rad, heartbeat=i, timestamp_s=i / ROBOT_HZ, **modes) for i in range(n)]


def endless_packets(q_rad: list[float] = START_Q_RAD) -> Iterator[dict[str, Any]]:
    i = 0
    while True:
        yield packet(q_rad, heartbeat=i)
        i += 1


def ticking_packets(q_rad: list[float] = START_Q_RAD) -> Iterator[dict[str, Any]]:
    """Leader quieto (misma `actual_q`) con el `timestamp` del controlador avanzando."""
    i = 0
    while True:
        yield packet(q_rad, heartbeat=0, timestamp_s=i / ROBOT_HZ)
        i += 1


class FakeRTDE:
    """Imita al cliente RTDE oficial (`rtde.rtde.RTDE`) en lo que usan follower y leader.

    El paquete `k` llega `(k + 1)/hz` después de `send_start`, con el reloj `clock` o el real.
    Una entrada `None` es un periodo sin paquete; al acabarse, lanza `RTDEException` como el
    cliente real al perder la conexión. `fail` activa por nombre los fallos del cliente real.
    """

    def __init__(
        self,
        packets: Iterable[dict[str, Any] | None],
        clock: FakeClock | None = None,
        hz: float = ROBOT_HZ,
        fail: Iterable[str] = (),
    ) -> None:
        self._packets = iter(packets)
        self._clock = clock
        self._period_s = 1 / hz
        self._fail = set(fail)
        self._input_names: dict[int, list[str]] = {}
        self.received = 0
        self._start_ns = 0
        self.connected = False
        self.started = False
        self.output_setup: tuple[list[str], list[str], float] | None = None
        self.input_setups: list[tuple[list[str], list[str]]] = []
        self.sent: list[dict[str, Any]] = []  # campos escritos en cada envío
        self.buffer_limits: list[int | None] = []  # `buffer_limit` de cada `receive_buffered`

    def connect(self) -> None:
        if "connect" in self._fail:
            raise ConnectionRefusedError("FakeRTDE: conexión rechazada")
        if "connect_protocol" in self._fail:
            self.connected = True  # el cliente real ya abrió el socket cuando falla la negociación
            raise RTDEException("Unable to negotiate protocol version")
        self.connected = True

    def disconnect(self) -> None:
        if "disconnect" in self._fail:
            raise OSError("FakeRTDE: fallo al cerrar el socket")
        self.connected = False
        self.started = False

    def is_connected(self) -> bool:
        return self.connected

    def send_output_setup(self, variables: list[str], types: list[str] = [], frequency: float = 125) -> bool:  # noqa: B006 - misma firma que el cliente real
        self._raise_if_setup_times_out()
        if "setup_oserror" in self._fail:
            raise ConnectionResetError("FakeRTDE: conexión reiniciada por el robot")
        if "send_output_setup" in self._fail:
            return False
        self.output_setup = (list(variables), list(types), frequency)
        return True

    def send_input_setup(self, variables: list[str], types: list[str] = []) -> SimpleNamespace | None:  # noqa: B006
        self._raise_if_setup_times_out()
        if "inputs_in_use" in self._fail:
            raise ValueError("An input parameter is already in use.")
        if "send_input_setup" in self._fail:
            return None
        recipe_id = len(self.input_setups) + 1
        self.input_setups.append((list(variables), list(types)))
        self._input_names[recipe_id] = list(variables)
        return SimpleNamespace(recipe_id=recipe_id, **dict.fromkeys(variables))

    def send_start(self) -> bool:
        if "start_lost" in self._fail:
            self.connected = False
            raise RTDEException(" _recv() Connection lost ")
        if "send_start" in self._fail:
            return False
        self.started = True
        self._start_ns = self._now_ns()
        return True

    def send(self, input_data: SimpleNamespace) -> bool | None:
        if not (self.connected and self.started):
            return None  # el cliente real registra el error y no envía
        if "send_oserror" in self._fail:
            raise ConnectionResetError("FakeRTDE: conexión reiniciada por el robot")
        fields = {name: getattr(input_data, name) for name in self._input_names[input_data.recipe_id]}
        for name, value in fields.items():
            if value is None:  # como `serialize.DataObject.pack` del cliente real
                raise ValueError("Uninitialized parameter: " + name)
        self.sent.append(fields)
        return True

    def receive(self) -> SimpleNamespace | None:
        if not self.started:
            raise RTDEException("Cannot receive when RTDE synchronization is inactive")
        if self._clock:
            self._clock.advance_s(self._period_s)
        else:
            time.sleep(self._period_s)
        pkt = self._next_packet()
        return None if pkt is None else SimpleNamespace(**pkt)

    def receive_buffered(self, binary: bool = False, buffer_limit: int | None = None) -> SimpleNamespace | None:
        self.buffer_limits.append(buffer_limit)
        while self.connected and self._arrival_ns(self.received) <= self._now_ns():
            pkt = self._next_packet()
            if pkt is not None:
                return SimpleNamespace(**pkt)
        return None

    def _next_packet(self) -> dict[str, Any] | None:
        self.received += 1
        try:
            return next(self._packets)
        except StopIteration:
            self.connected = False
            raise RTDEException(" _recv() Connection lost ") from None

    def _arrival_ns(self, index: int) -> int:
        return self._start_ns + round((index + 1) * self._period_s * 1e9)

    def _now_ns(self) -> int:
        return self._clock() if self._clock else now_ns()

    def _raise_if_setup_times_out(self) -> None:
        if "setup_timeout" in self._fail:
            # El mismo error que el cliente real al hacer `result.types` con result=None.
            raise AttributeError("'NoneType' object has no attribute 'types'")

    # --- consultas para los asserts -----------------------------------------------------
    def written_enable(self) -> list[int]:
        return [fields["input_int_register_0"] for fields in self.sent if "input_int_register_0" in fields]
