"""Dobles de test del follower UR: RTDE, Dashboard, interfaz secundaria y reloj.

Los paquetes RTDE son dicts con los campos de la receta de salida; así la misma `FakeRTDE`
sirve para paquetes escritos a mano y para trazas grabadas en URSim.
"""

import math
import time
from collections.abc import Callable, Iterable, Iterator
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


class FakeClock:
    """Reloj en ns que solo avanza cuando el test (o `FakeRTDE`) lo pide: nada duerme."""

    def __init__(self, start_ns: int = 1_000_000_000) -> None:
        self.now_ns = start_ns

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


def packets(n: int, q_rad: list[float] = START_Q_RAD, hz: float = 125, **modes: int) -> list[dict[str, Any]]:
    """`n` paquetes de un robot quieto en `q_rad` con el heartbeat latiendo."""
    return [packet(q_rad, heartbeat=i, timestamp_s=i / hz, **modes) for i in range(n)]


def endless_packets(q_rad: list[float] = START_Q_RAD) -> Iterator[dict[str, Any]]:
    i = 0
    while True:
        yield packet(q_rad, heartbeat=i)
        i += 1


class FakeRTDE:
    """Imita al cliente RTDE oficial (`rtde.rtde.RTDE`) en lo que usa el follower.

    - `receive()` entrega el siguiente paquete (un dict, o `None` = timeout del cliente real).
      Con `clock` avanza ese reloj `1/hz`; sin él, duerme `1/hz` de verdad (tests con hilos).
      Al acabarse los paquetes lanza `RTDEException`, como el cliente real al perder la conexión.
    - `actions[k]` se ejecuta justo antes de entregar el paquete `k` (0 = el primero).
    - `fail` simula los fallos que el cliente real devuelve como valor, no como excepción:
      `send_input_setup` → `None`, `send_output_setup`/`send_start` → `False`.
      Y los que lanza: `connect` → `ConnectionRefusedError`; `connect_protocol` → `RTDEException`
      con el socket ya abierto (el puerto acepta pero el controlador no negocia, p. ej. URSim
      arrancando); `setup_timeout` → `AttributeError` en `send_output_setup`/`send_input_setup`
      (el cliente real hace `result.types` sobre una respuesta que no llegó); `inputs_in_use` →
      `ValueError` en `send_input_setup` (el cliente real al ver `IN_USE` en la respuesta);
      `start_lost` → `RTDEException` en `send_start` (conexión perdida en plena configuración);
      `setup_oserror` → `ConnectionResetError` en `send_output_setup` y `send_oserror` → lo mismo
      en `send` (lo lanza `sock.sendall`); `disconnect` → `OSError` en `disconnect` (lo lanza
      `sock.close` del cliente real).
    - `send()` sin sincronización activa (antes de `send_start`, tras `disconnect` o tras perder la
      conexión) no envía y devuelve None; con un campo de la receta sin valor lanza ValueError.
    """

    def __init__(
        self,
        packets: Iterable[dict[str, Any] | None],
        clock: FakeClock | None = None,
        hz: float = 125,
        actions: dict[int, Callable[[], None]] | None = None,
        fail: Iterable[str] = (),
    ) -> None:
        self._packets = iter(packets)
        self._clock = clock
        self._period_s = 1 / hz
        self._actions = actions or {}
        self._fail = set(fail)
        self._input_names: dict[int, list[str]] = {}
        self.received = 0
        self.connected = False
        self.started = False
        self.output_setup: tuple[list[str], list[str], float] | None = None
        self.input_setups: list[tuple[list[str], list[str]]] = []
        self.sent: list[tuple[int, dict[str, Any]]] = []  # (t_ns, campos escritos)

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
        return True

    def send_pause(self) -> bool:
        self.started = False
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
        t_ns = self._clock() if self._clock else now_ns()
        self.sent.append((t_ns, fields))
        return True

    def receive(self) -> SimpleNamespace | None:
        if not self.started:
            raise RTDEException("Cannot receive when RTDE synchronization is inactive")
        action = self._actions.get(self.received)
        if action is not None:
            action()
        self.received += 1
        if self._clock:
            self._clock.advance_s(self._period_s)
        else:
            time.sleep(self._period_s)
        try:
            pkt = next(self._packets)
        except StopIteration:
            self.connected = False
            raise RTDEException(" _recv() Connection lost ") from None
        return None if pkt is None else SimpleNamespace(**pkt)

    def _raise_if_setup_times_out(self) -> None:
        if "setup_timeout" in self._fail:
            # El mismo error que el cliente real al hacer `result.types` con result=None.
            raise AttributeError("'NoneType' object has no attribute 'types'")

    # --- consultas para los asserts -----------------------------------------------------
    def written_q(self) -> list[list[float]]:
        """Consignas articulares escritas, en orden."""
        return [
            [fields[f"input_double_register_{i}"] for i in range(6)]
            for _, fields in self.sent
            if "input_double_register_0" in fields
        ]

    def written_enable(self) -> list[int]:
        return [fields["input_int_register_0"] for _, fields in self.sent if "input_int_register_0" in fields]


class FakeDashboard:
    """Dashboard (29999) con respuestas preparadas; guarda los comandos recibidos.

    Como `DashboardClient`, `connect()` no hace nada si ya hay conexión; `connections` cuenta las
    que se abren de verdad. `stop` contesta "Stopped", como el Dashboard de UR, salvo otra respuesta.
    `fail`: `connect` → `ConnectionRefusedError`, `send` → `OSError` y `close` → `OSError` (lo
    lanza `sock.close` del cliente real).
    """

    def __init__(self, responses: dict[str, str] | None = None, fail: Iterable[str] = ()) -> None:
        self._responses = {"stop": "Stopped", **(responses or {})}
        self._fail = set(fail)
        self.commands: list[str] = []
        self.connected = False
        self.connections = 0

    def connect(self) -> None:
        if self.connected:
            return
        if "connect" in self._fail:
            raise ConnectionRefusedError("FakeDashboard: conexión rechazada")
        self.connected = True
        self.connections += 1

    def send(self, command: str) -> str:
        if "send" in self._fail:
            raise OSError("FakeDashboard: fallo al enviar")
        assert self.connected, "send() sin connect()"
        self.commands.append(command)
        return self._responses.get(command, "")

    def close(self) -> None:
        if "close" in self._fail:
            raise OSError("FakeDashboard: fallo al cerrar")
        self.connected = False


class FakeSecondary:
    """Interfaz secundaria (30002): guarda los scripts subidos en lugar de enviarlos."""

    def __init__(self) -> None:
        self.scripts: list[tuple[str, str]] = []  # (host, texto)

    def __call__(self, host: str, script_text: str) -> None:
        self.scripts.append((host, script_text))
