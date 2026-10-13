"""Follower UR visto desde el proceso de LeRobot: escribe consignas y lee el estado del robot.

El programa del TP interpola, limita la velocidad y se queda en la última consigna; aquí solo se
escribe cada consigna al momento y se lee el estado cuando LeRobot lo pide. Todo corre en el hilo
que llama, sin hilos ni procesos: entre `connect()` y `disconnect()` nunca se espera al robot
(`receive_buffered` no bloquea), así que el ritmo lo marca LeRobot. Es una segunda capa: la seguridad real es la de
PolyScope.
"""

import logging
import math
from collections.abc import Callable, Sequence
from typing import Any

from rtde.rtde import RTDEException

from ur_core.clock import now_ns
from ur_core.config import N_JOINTS, TeleopConfig
from ur_core.startup import RtdeConnection, check_pose, rtde_connection, start_pose_text

log = logging.getLogger("ur_core.follower")

# El robot manda su estado cada 8 ms: leyendo a ese ritmo, la pose que graba LeRobot tiene como
# mucho 8 ms.
READ_HZ = 125
# Tiempo interno de arranque, no un ajuste del operador: que el programa del TP empiece a latir.
ARM_TIMEOUT_S = 6.0
# Sin paquete nuevo en este tiempo, la conexión está perdida aunque siga abierta (cable caído).
STALE_S = 1.0

OUTPUT_NAMES = [
    "actual_q",
    "timestamp",
    "robot_mode",
    "safety_mode",
    "runtime_state",
    "output_int_register_0",
]
OUTPUT_TYPES = ["VECTOR6D", "DOUBLE", "INT32", "INT32", "UINT32", "INT32"]
SETPOINT_NAMES = [f"input_double_register_{i}" for i in range(N_JOINTS)]
# Una sola receta de entrada: consigna, enable y contador llegan siempre juntos al robot.
# El contador cambia en cada escritura: el programa del TP toma cada cambio como una consigna
# nueva (no espera a un valor, porque los registros guardan el último tras desconectar).
COMMAND_NAMES = [*SETPOINT_NAMES, "input_int_register_0", "input_int_register_1"]
COMMAND_TYPES = ["DOUBLE"] * N_JOINTS + ["INT32", "INT32"]
INT32_LIMIT = 2**31

# Valores de RTDE (guía de RTDE de Universal Robots), con nombre para que los motivos se lean.
ROBOT_MODES = {
    -1: "NO_CONTROLLER",
    0: "DISCONNECTED",
    1: "CONFIRM_SAFETY",
    2: "BOOTING",
    3: "POWER_OFF",
    4: "POWER_ON",
    5: "IDLE",
    6: "BACKDRIVE",
    7: "RUNNING",
    8: "UPDATING_FIRMWARE",
}
SAFETY_MODES = {
    1: "NORMAL",
    2: "REDUCED",
    3: "PROTECTIVE_STOP",
    4: "RECOVERY",
    5: "SAFEGUARD_STOP",
    6: "SYSTEM_EMERGENCY_STOP",
    7: "ROBOT_EMERGENCY_STOP",
    8: "VIOLATION",
    9: "FAULT",
    10: "VALIDATE_JOINT_ID",
    11: "UNDEFINED_SAFETY_MODE",
    12: "AUTOMATIC_MODE_SAFEGUARD_STOP",
    13: "SYSTEM_THREE_POSITION_ENABLING_STOP",
}
RUNTIME_STATES = {0: "STOPPING", 1: "STOPPED", 2: "PLAYING", 3: "PAUSING", 4: "PAUSED", 5: "RESUMING"}
ROBOT_MODE_RUNNING = 7
SAFETY_MODES_OK = (1, 2)  # REDUCED es funcionamiento normal con límites más bajos
RUNTIME_PLAYING = 2


class FollowerStartError(RuntimeError):
    """El follower no llegó a quedar armado; el motivo dice qué falló."""


class FollowerStoppedError(RuntimeError):
    """El follower ya paró (motivo en el mensaje); hay que dar Play en el TP y relanzar el comando."""


class _ConnectionLostError(Exception):
    """Interna: al armar es un fallo de arranque, en teleoperación una parada y en `disconnect` se registra."""


def _named(table: dict[int, str], value: int) -> str:
    return f"{table.get(value, '?')} ({value})"


class UrFollowerCore:
    def __init__(
        self,
        config: TeleopConfig,
        rtde_factory: Callable[[str], RtdeConnection] = rtde_connection,
        clock: Callable[[], int] = now_ns,
    ) -> None:
        self._config = config
        self._rtde_factory = rtde_factory
        self._clock = clock
        self._rtde: RtdeConnection | None = None
        self._command: Any = None
        self._counter = 0
        self._last_pkt: Any = None
        self._last_rx_ns = 0
        self._last_q_rad: list[float] = []  # última consigna escrita
        self._enabled = False
        self._stop_reason = ""

    @property
    def is_connected(self) -> bool:
        return self._rtde is not None

    # --- arranque -----------------------------------------------------------------------
    def start_prompt(self) -> str:
        follower = self._config.follower
        pose = start_pose_text(self._config.start_pose_rad, self._config.start_tolerance_rad)
        return (
            f"Follower {follower.type} ({follower.ip}), desde el Teach Pendant:\n"
            "  - Programa del follower cargado y en marcha (Play).\n"
            f"  - Robot en la posición inicial {pose} (base..wrist_3).\n"
            # Termina en salto de línea: con `| Tee-Object`, PowerShell no muestra una línea sin él.
            "¿Continuar? (yes/no):\n"
        )

    def check_start(self) -> list[str]:
        """Problemas que impiden arrancar ([] = listo). No lanza: cada fallo es una línea.

        Que el programa del TP esté en marcha no se mira aquí: lo comprueba `connect()` al ver su
        heartbeat, que es la única prueba de que corre el nuestro.
        """
        follower = self._config.follower
        return check_pose(
            "follower",
            follower.ip,
            self._rtde_factory(follower.ip),
            READ_HZ,
            self._config.start_pose_rad,
            self._config.start_tolerance_rad,
        )

    def connect(self) -> None:
        """Abre RTDE, escribe la pose medida deshabilitada y vuelve cuando late el programa del TP."""
        if self._rtde is not None:
            return
        self._rtde = self._rtde_factory(self._config.follower.ip)
        self._last_q_rad, self._enabled, self._stop_reason = [], False, ""
        try:
            self._arm()
        except BaseException:
            # El cliente oficial deja el socket abierto si falla la negociación.
            self._close()
            raise
        log.info("follower: conectado y armado t_ns=%d", self._clock())

    def _arm(self) -> None:
        rtde, ip = self._connection(), self._config.follower.ip
        try:
            rtde.connect()
        except (RTDEException, OSError) as exc:
            # RTDEException: el puerto acepta pero el controlador no negocia el protocolo (Docker
            # publica el puerto aunque URSim aún esté arrancando).
            raise FollowerStartError(
                f"no se puede conectar por RTDE a {ip}: {exc} (¿el controlador todavía está arrancando?)"
            ) from exc
        if not self._rtde_setup(rtde.send_output_setup, OUTPUT_NAMES, OUTPUT_TYPES, frequency=READ_HZ):
            raise FollowerStartError(f"RTDE: el robot rechazó la receta de salida {OUTPUT_NAMES}")
        self._command = self._input_setup()
        if not self._rtde_setup(rtde.send_start):
            raise FollowerStartError("RTDE: no se pudo iniciar la sincronización")
        pkt = self._receive_arming()
        self._check_ready(pkt)
        try:
            # Consigna = pose actual y deshabilitado desde la primera escritura: nunca hay salto.
            self._write(pkt.actual_q, enable=0)
        except _ConnectionLostError as exc:
            raise FollowerStartError(str(exc)) from exc
        self._wait_for_heartbeat(pkt)

    def _wait_for_heartbeat(self, pkt: Any) -> None:
        hb0 = pkt.output_int_register_0
        deadline_ns = self._clock() + round(ARM_TIMEOUT_S * 1e9)
        while pkt.output_int_register_0 == hb0:
            if self._clock() > deadline_ns:
                raise FollowerStartError(
                    "el programa del follower no está en marcha: dale a Play en el TP "
                    f"(heartbeat sin cambios en {ARM_TIMEOUT_S} s)"
                )
            pkt = self._receive_arming()
        self._last_pkt, self._last_rx_ns = pkt, self._clock()

    def _receive_arming(self) -> Any:
        """Solo durante el armado se espera al robot: después nunca se llama a `receive()`."""
        ip = self._config.follower.ip
        try:
            pkt = self._connection().receive()
        except (RTDEException, OSError) as exc:
            raise FollowerStartError(
                f"RTDE: se perdió la conexión con {ip} durante el armado: {exc}"
            ) from exc
        if pkt is None:  # el cliente oficial devuelve None tras su DEFAULT_TIMEOUT sin datos
            raise FollowerStartError(f"RTDE: sin datos del robot {ip} durante el armado")
        return pkt

    def _rtde_setup(self, call: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
        """Una llamada de configuración del cliente oficial, con sus fallos como FollowerStartError.

        Envuelve solo la llamada a la librería: un AttributeError de nuestro código no es "el robot
        no respondió", es un error nuestro y tiene que verse como tal.
        """
        ip = self._config.follower.ip
        try:
            return call(*args, **kwargs)
        except AttributeError as exc:
            # El cliente oficial hace `result.types` sobre la respuesta sin comprobar que llegó:
            # si el robot no contesta a tiempo lanza AttributeError en vez de devolver un error.
            raise FollowerStartError(f"el robot {ip} no respondió a la configuración RTDE") from exc
        except (RTDEException, OSError) as exc:
            # `sock.sendall` lanza OSError; el cliente lanza RTDEException si el robot cierra.
            raise FollowerStartError(
                f"RTDE: se perdió la conexión con {ip} durante la configuración: {exc}"
            ) from exc

    def _input_setup(self) -> Any:
        busy = (
            "RTDE: registros de entrada input_double_register_0..5 / input_int_register_0..1 "
            "no disponibles (¿los usa otro cliente RTDE o un bus de campo?)"
        )
        try:
            command = self._rtde_setup(self._connection().send_input_setup, COMMAND_NAMES, COMMAND_TYPES)
        except ValueError as exc:
            # El cliente oficial lanza ValueError al leer `IN_USE` en la respuesta, en vez de
            # devolver None. Solo aquí: en la receta de salida significaría otra cosa.
            raise FollowerStartError(f"{busy}: {exc}") from exc
        if command is None:  # tipos distintos a los pedidos
            raise FollowerStartError(busy)
        return command

    @staticmethod
    def _check_ready(pkt: Any) -> None:
        # `runtime_state` no: el programa del TP puede arrancar mientras se espera su heartbeat.
        if pkt.robot_mode != ROBOT_MODE_RUNNING:
            raise FollowerStartError(
                f"el follower no está listo: robot_mode={_named(ROBOT_MODES, pkt.robot_mode)} "
                "(encender y soltar frenos desde el Teach Pendant)"
            )
        if pkt.safety_mode not in SAFETY_MODES_OK:
            raise FollowerStartError(
                f"el follower no está listo: safety_mode={_named(SAFETY_MODES, pkt.safety_mode)}"
            )

    # --- teleoperación ------------------------------------------------------------------
    def send_joints(self, q_rad: Sequence[float]) -> None:
        """Escribe la consigna (6 rad) con enable=1 en un único paquete; no espera al robot."""
        target_rad = [float(x) for x in q_rad]
        if len(target_rad) != N_JOINTS:
            raise ValueError(f"se esperaban {N_JOINTS} articulaciones y llegan {len(target_rad)}")
        if not all(math.isfinite(x) for x in target_rad):
            raise ValueError(f"consigna con valores no finitos: {target_rad}")
        pkt = self._read_state()
        if not self._enabled:
            self._check_first_target(target_rad, pkt.actual_q)
        try:
            self._write(target_rad, enable=1)
        except _ConnectionLostError as exc:
            raise self._stop(str(exc)) from exc
        if not self._enabled:
            self._enabled = True
            log.info("follower: habilitado t_ns=%d", self._clock())

    def get_joints(self) -> list[float]:
        """Última `actual_q` medida, en rad.

        Tras una parada lanza en vez de devolver la última pose: ya no se actualiza, y una pose
        congelada haría que LeRobot siguiera grabando con el robot parado.
        """
        return list(self._read_state().actual_q)

    def _check_first_target(self, target_rad: list[float], measured_rad: Sequence[float]) -> None:
        error_rad = max(abs(t - m) for t, m in zip(target_rad, measured_rad, strict=True))
        # Leader y follower pasan cada uno el arranque a ± tolerancia: entre sí, hasta el doble.
        max_error_rad = 2 * self._config.start_tolerance_rad
        if error_rad > max_error_rad:
            raise self._stop(
                f"primera consigna lejos de la posición actual "
                f"({math.degrees(error_rad):.1f}° > {math.degrees(max_error_rad):.1f}°)"
            )

    def _read_state(self) -> Any:
        """Vacía lo acumulado y comprueba el paquete más nuevo. Tras una parada, lanza siempre."""
        self._connection()
        if self._stop_reason:
            raise FollowerStoppedError(self._stop_reason)
        try:
            self._drain()
        except _ConnectionLostError as exc:
            raise self._stop(str(exc)) from exc
        reason = self._robot_fault(self._last_pkt)
        if reason:
            raise self._stop(reason)
        return self._last_pkt

    def _drain(self) -> None:
        """Se queda con el último paquete llegado. La edad se mide después de vaciar: tras una
        pausa larga llegan todos los acumulados y no es una conexión perdida."""
        rtde = self._connection()
        newest = None
        try:
            while (pkt := rtde.receive_buffered()) is not None:
                newest = pkt
        except (RTDEException, OSError) as exc:
            raise _ConnectionLostError(f"se perdió la conexión RTDE: {exc}") from exc
        if newest is not None:
            self._last_pkt, self._last_rx_ns = newest, self._clock()
        # El cliente oficial entrega lo que quedaba en su buffer aunque el robot ya cerrara.
        if not rtde.is_connected():
            raise _ConnectionLostError("se perdió la conexión RTDE: conexión cerrada")
        age_ns = self._clock() - self._last_rx_ns
        if age_ns > STALE_S * 1e9:
            raise _ConnectionLostError(
                f"se perdió la conexión RTDE: ningún paquete nuevo en {age_ns / 1e9:.1f} s"
            )

    @staticmethod
    def _robot_fault(pkt: Any) -> str:
        if pkt.safety_mode not in SAFETY_MODES_OK:
            return f"parada de seguridad del follower: safety_mode={_named(SAFETY_MODES, pkt.safety_mode)}"
        if pkt.robot_mode != ROBOT_MODE_RUNNING:
            return f"el follower dejó de estar en marcha: robot_mode={_named(ROBOT_MODES, pkt.robot_mode)}"
        if pkt.runtime_state != RUNTIME_PLAYING:
            return (
                "el programa del follower no está en ejecución: "
                f"runtime_state={_named(RUNTIME_STATES, pkt.runtime_state)}"
            )
        return ""

    def _stop(self, reason: str) -> FollowerStoppedError:
        """La primera parada es definitiva: desde aquí todas las llamadas lanzan con su motivo."""
        self._stop_reason = reason
        log.error("follower: parado t_ns=%d %s", self._clock(), reason)
        return FollowerStoppedError(reason)

    def _write(self, q_rad: Sequence[float], enable: int) -> None:
        for name, value in zip(SETPOINT_NAMES, q_rad, strict=True):
            setattr(self._command, name, value)
        self._command.input_int_register_0 = enable
        self._counter = (self._counter + 1) % INT32_LIMIT
        self._command.input_int_register_1 = self._counter
        # El cliente oficial devuelve None si no puede enviar, pero `sock.sendall` sí lanza
        # OSError (p. ej. conexión reiniciada).
        try:
            sent = self._connection().send(self._command)
        except OSError as exc:
            raise _ConnectionLostError(f"se perdió la conexión RTDE: {exc}") from exc
        if not sent:
            raise _ConnectionLostError("se perdió la conexión RTDE: no se pudo escribir la consigna")
        self._last_q_rad = list(q_rad)

    # --- parada -------------------------------------------------------------------------
    def disconnect(self) -> None:
        """enable=0 (el programa del TP frena con `stopj` y termina) y cierra RTDE. Idempotente.

        No lanza: va en el `finally` de LeRobot, antes de cerrar el leader y las cámaras.
        """
        if self._rtde is None:
            return
        try:
            if self._rtde.is_connected():
                # La receta lleva siempre la consigna: repetir la última no pide ningún movimiento.
                self._write(self._last_q_rad, enable=0)
                log.info("follower: parada pedida (disconnect) t_ns=%d", self._clock())
        except _ConnectionLostError as exc:
            log.error(
                "follower: no se pudo escribir enable=0 (Stop en el TP) t_ns=%d: %s", self._clock(), exc
            )
        finally:
            self._close()

    def _close(self) -> None:
        """Cierra RTDE y olvida la conexión pase lo que pase.

        `sock.close` del cliente oficial puede lanzar OSError: se registra y no se propaga, para
        no tapar el error de arranque ni impedir que LeRobot cierre el leader y las cámaras.
        """
        try:
            self._connection().disconnect()
        except OSError:
            log.exception("follower: fallo al cerrar la conexión RTDE t_ns=%d", self._clock())
        finally:
            self._rtde = None

    def _connection(self) -> RtdeConnection:
        if self._rtde is None:
            raise RuntimeError("follower no conectado: llama a connect()")
        return self._rtde
