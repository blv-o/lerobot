"""Leader UR: se lee por RTDE y nunca se le escribe nada.

El operador lo mueve a mano en freedrive desde un programa del TP. Solo hay una receta de
salida: ninguna de entrada, ningún script por 30002 y ningún comando de Dashboard, para no
interferir con lo que corre en el robot. Como el follower, todo corre en el hilo que llama, sin
hilos propios: tras `connect()` nunca se espera al robot (`receive_buffered` no bloquea).
"""

import logging
from collections.abc import Callable

from rtde.rtde import RTDEException

from ur_core.clock import now_ns
from ur_core.config import UR_TYPES, ConfigError, TeleopConfig, UrLeaderConfig
from ur_core.startup import (
    BUFFER_LIMIT,
    READ_HZ,
    RtdeConnection,
    check_pose,
    receive_packet,
    rtde_connection,
    start_output,
    start_pose_text,
)

log = logging.getLogger("ur_core.leader")

# Toda causa de un error del leader está en su TP o en la red.
READ_ACTION = "comprueba el programa de freedrive y el cable del leader"

OUTPUT_NAMES = ["actual_q", "timestamp"]
OUTPUT_TYPES = ["VECTOR6D", "DOUBLE"]


class LeaderReadError(RuntimeError):
    """El leader no da datos nuevos (conexión perdida o `timestamp` parado); el motivo lo dice."""


class UrLeaderCore:
    def __init__(
        self,
        config: TeleopConfig,
        rtde_factory: Callable[[str], RtdeConnection] = rtde_connection,
        clock: Callable[[], int] = now_ns,
    ) -> None:
        if not isinstance(config.leader, UrLeaderConfig):
            raise ConfigError(
                "leader.type",
                f"{config.leader.type!r} no es un UR: este leader solo lee modelos {list(UR_TYPES)}",
            )
        self._config = config
        self._leader = config.leader
        self._rtde_factory = rtde_factory
        self._clock = clock
        self._timeout_ns = round(config.leader.timeout_s * 1e9)
        self._rtde: RtdeConnection | None = None
        self._q_rad: list[float] = []
        self._timestamp_s = 0.0
        self._advanced_ns = 0  # cuándo avanzó por última vez el `timestamp` del controlador

    @property
    def is_connected(self) -> bool:
        return self._rtde is not None

    # --- arranque -----------------------------------------------------------------------
    def start_prompt(self) -> str:
        pose = start_pose_text(self._config.start_pose_rad, self._config.start_tolerance_rad)
        return (
            f"Leader {self._leader.type} ({self._leader.ip}), desde el Teach Pendant:\n"
            "  - Programa de freedrive cargado y en marcha (Play).\n"
            f"  - Robot en la posición inicial {pose} (base..wrist_3).\n"
            # Termina en salto de línea: con `| Tee-Object`, PowerShell no muestra una línea sin él.
            "¿Continuar? (yes/no):\n"
        )

    def check_start(self) -> list[str]:
        """Problemas que impiden arrancar ([] = listo): conexión RTDE y posición inicial. No lanza."""
        return check_pose(
            "leader",
            self._leader.ip,
            self._rtde_factory(self._leader.ip),
            READ_HZ,
            self._config.start_pose_rad,
            self._config.start_tolerance_rad,
        )

    def connect(self) -> None:
        """Abre la lectura; `read_joints()` ya da la pose real al volver."""
        if self._rtde is not None:
            return
        ip = self._leader.ip
        con = self._rtde_factory(ip)
        try:
            start_output(con, OUTPUT_NAMES, OUTPUT_TYPES, READ_HZ)
            pkt = receive_packet(con)
        except (OSError, RTDEException) as exc:
            # El cliente oficial deja el socket abierto si falla la negociación.
            self._close(con)
            raise self._error(f"no se puede leer el leader por RTDE en {ip}: {exc}") from exc
        self._rtde = con
        self._q_rad, self._timestamp_s, self._advanced_ns = list(pkt.actual_q), pkt.timestamp, self._clock()
        log.info("leader: conectado t_ns=%d", self._clock())

    # --- lectura ------------------------------------------------------------------------
    def read_joints(self) -> list[float]:
        """Última `actual_q` del leader, en rad: vacía lo acumulado y no espera al robot."""
        con = self._connection()
        lost = f"se perdió la conexión RTDE con el leader {self._leader.ip}"
        try:
            while (pkt := con.receive_buffered(buffer_limit=BUFFER_LIMIT)) is not None:
                # Cuenta el `timestamp` del controlador y no `actual_q`: en freedrive el leader
                # puede estar quieto.
                if pkt.timestamp > self._timestamp_s:
                    self._q_rad, self._timestamp_s = list(pkt.actual_q), pkt.timestamp
                    self._advanced_ns = self._clock()
        except (OSError, RTDEException) as exc:
            raise self._error(f"{lost}: {exc}") from exc
        # El cliente oficial entrega lo que quedaba en su buffer aunque el robot ya cerrara.
        if not con.is_connected():
            raise self._error(f"{lost}: conexión cerrada")
        age_ns = self._clock() - self._advanced_ns
        if age_ns > self._timeout_ns:
            raise self._error(
                f"el leader no da datos nuevos: timestamp parado {age_ns / 1e6:.0f} ms "
                f"(límite {self._leader.timeout_s * 1000:g} ms)"
            )
        return list(self._q_rad)

    def _error(self, reason: str) -> LeaderReadError:
        reason = f"{reason}: {READ_ACTION}"
        log.error("leader: t_ns=%d %s", self._clock(), reason)
        return LeaderReadError(reason)

    def disconnect(self) -> None:
        """Cierra la conexión. Idempotente y sin lanzar: va en el `finally` de LeRobot."""
        if self._rtde is None:
            return
        try:
            self._close(self._rtde)
        finally:
            self._rtde = None
            log.info("leader: cerrado t_ns=%d", self._clock())

    def _close(self, con: RtdeConnection) -> None:
        """`sock.close` del cliente oficial puede lanzar: se registra, para no tapar el error de
        conexión ni impedir que LeRobot siga cerrando."""
        try:
            con.disconnect()
        except OSError:
            log.exception("leader: fallo al cerrar la conexión RTDE t_ns=%d", self._clock())

    def _connection(self) -> RtdeConnection:
        if self._rtde is None:
            raise RuntimeError("leader no conectado: llama a connect()")
        return self._rtde
