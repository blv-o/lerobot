"""Leader UR: se lee por RTDE y nunca se le escribe nada.

El operador lo mueve a mano en freedrive desde un programa del TP. Solo hay una receta de
salida: ninguna de entrada, ningún script por 30002 y ningún comando de Dashboard, para no
interferir con lo que corre en el robot.
"""

import logging
import threading
from collections.abc import Callable

import rtde.rtde as rtde
from rtde.rtde import RTDEException

from ur_core.clock import now_ns
from ur_core.config import UR_TYPES, ConfigError, TeleopConfig, UrLeaderConfig
from ur_core.startup import check_pose, receive_packet, start_output, start_pose_text
from ur_core.streaming import RTDE_PORT, RtdeConnection

log = logging.getLogger("ur_core.leader")

OUTPUT_NAMES = ["actual_q", "timestamp"]
OUTPUT_TYPES = ["VECTOR6D", "DOUBLE"]
# `receive()` del cliente oficial espera como mucho `rtde.DEFAULT_TIMEOUT`: el hilo ve la parada
# después de ese tiempo, como muy tarde.
JOIN_TIMEOUT_S = rtde.DEFAULT_TIMEOUT + 1.0


class LeaderReadError(RuntimeError):
    """El leader no da datos nuevos (conexión perdida o `timestamp` parado); el motivo lo dice."""


def _rtde_connection(ip: str) -> RtdeConnection:
    return rtde.RTDE(ip, RTDE_PORT)


class UrLeaderCore:
    def __init__(
        self, config: TeleopConfig, rtde_factory: Callable[[str], RtdeConnection] = _rtde_connection
    ) -> None:
        if not isinstance(config.leader, UrLeaderConfig):
            raise ConfigError(
                "leader.type",
                f"{config.leader.type!r} no es un UR: este leader solo lee modelos {list(UR_TYPES)}",
            )
        self._config = config
        self._leader = config.leader
        self._rtde_factory = rtde_factory
        self._timeout_ns = round(config.leader.timeout_s * 1e9)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()  # uno nuevo por conexión: un hilo viejo nunca revive
        # Una sola tupla (actual_q, instante en que avanzó el timestamp): se reemplaza entera, así
        # quien lee nunca ve una pose de un paquete con el instante de otro.
        self._latest: tuple[list[float], int] = ([], 0)
        self._error = ""

    @property
    def is_connected(self) -> bool:
        return self._thread is not None

    # --- arranque -----------------------------------------------------------------------
    def start_prompt(self) -> str:
        pose = start_pose_text(self._config.start_pose_rad, self._config.start_tolerance_rad)
        return (
            f"Leader {self._leader.type} ({self._leader.ip}), desde el Teach Pendant:\n"
            "  - Programa de freedrive cargado y en marcha (Play).\n"
            f"  - Robot en la posición inicial {pose} (base..wrist_3).\n"
            "¿Continuar? (yes/no): "
        )

    def check_start(self) -> list[str]:
        """Problemas que impiden arrancar ([] = listo): conexión RTDE y posición inicial. No lanza."""
        return check_pose(
            "leader",
            self._leader.ip,
            self._rtde_factory(self._leader.ip),
            self._leader.rtde_hz,
            self._config.start_pose_rad,
            self._config.start_tolerance_rad,
        )

    def connect(self) -> None:
        """Abre la lectura y lanza el hilo; `read_joints()` ya da la pose real al volver."""
        if self._thread is not None:
            return
        ip = self._leader.ip
        con = self._rtde_factory(ip)
        try:
            start_output(con, OUTPUT_NAMES, OUTPUT_TYPES, self._leader.rtde_hz)
            pkt = receive_packet(con)
        except (OSError, RTDEException) as exc:
            con.disconnect()  # el cliente oficial deja el socket abierto si falla la negociación
            raise LeaderReadError(f"no se puede leer el leader por RTDE en {ip}: {exc}") from exc
        self._latest, self._error = (list(pkt.actual_q), now_ns()), ""
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._read_loop, args=(con, pkt.timestamp, self._stop), name="ur-leader-rtde", daemon=True
        )
        self._thread.start()

    # --- lectura ------------------------------------------------------------------------
    def read_joints(self) -> list[float]:
        """Última `actual_q` del leader, en rad, sin esperar al robot.

        La edad se mide aquí y no en el hilo: el hilo puede pasar hasta `rtde.DEFAULT_TIMEOUT`
        dentro de `receive()`, mucho más que `timeout_ms`.
        """
        if self._thread is None:
            raise RuntimeError("leader no conectado: llama a connect()")
        q_rad, advanced_ns = self._latest
        if self._error:
            raise LeaderReadError(self._error)
        age_ns = now_ns() - advanced_ns
        if age_ns > self._timeout_ns:
            raise LeaderReadError(
                f"el leader no da datos nuevos: timestamp parado {age_ns / 1e6:.0f} ms "
                f"(límite {self._leader.timeout_s * 1000:g} ms)"
            )
        return list(q_rad)

    def _read_loop(self, con: RtdeConnection, timestamp_s: float, stop: threading.Event) -> None:
        """Único dueño de la conexión tras `connect()`: cerrarla desde otro hilo corta su `receive()`."""
        try:
            while not stop.is_set():
                pkt = con.receive()
                # Cuenta el `timestamp` del controlador y no `actual_q`: en freedrive el leader puede
                # estar quieto. None = el cliente oficial no recibió nada en su DEFAULT_TIMEOUT.
                if pkt is not None and pkt.timestamp > timestamp_s:
                    timestamp_s = pkt.timestamp
                    self._latest = (list(pkt.actual_q), now_ns())
        except (OSError, RTDEException) as exc:
            self._error = f"se perdió la conexión RTDE con el leader {self._leader.ip}: {exc}"
            log.error("leader: %s", self._error)
        except Exception as exc:
            log.exception("leader: error inesperado en el hilo de lectura")
            self._error = f"error inesperado leyendo el leader: {exc!r}"
        finally:
            con.disconnect()

    def disconnect(self) -> None:
        """Para el hilo de lectura, que cierra la conexión. Idempotente."""
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=JOIN_TIMEOUT_S)
        if self._thread.is_alive():
            log.error(
                "leader: el hilo de lectura no terminó en %s s; se abandona (es daemon)", JOIN_TIMEOUT_S
            )
        self._thread = None


LEADERS: dict[str, type[UrLeaderCore]] = dict.fromkeys(UR_TYPES, UrLeaderCore)
