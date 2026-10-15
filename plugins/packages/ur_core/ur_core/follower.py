"""Follower UR visto desde el proceso de LeRobot: escribe consignas y lee el estado del robot.

El programa del TP interpola, limita la velocidad y se queda en la última consigna; aquí solo se
escribe cada consigna al momento y se lee el estado cuando LeRobot lo pide. Todo corre en el hilo
que llama, sin hilos ni procesos: tras `connect()` nunca se espera al robot (`receive_buffered`
no bloquea), así que el ritmo lo marca LeRobot. Es una segunda capa: la seguridad real es la de
PolyScope.
"""

import logging
import math
from collections.abc import Callable, Sequence
from typing import Any

from rtde.rtde import RTDEException

from ur_core.clock import now_ns
from ur_core.config import N_JOINTS, TeleopConfig
from ur_core.startup import (
    BUFFER_LIMIT,
    READ_HZ,
    RtdeConnection,
    check_pose,
    rtde_connection,
    start_output,
    start_pose_text,
)

log = logging.getLogger("ur_core.follower")

# Tiempo interno de arranque, no un ajuste del operador: que el programa del TP empiece a latir.
ARM_TIMEOUT_S = 6.0
# Sin paquete nuevo en este tiempo, la conexión está perdida aunque siga abierta (cable caído).
STALE_S = 1.0
# Tras cualquier parada el programa del TP ya no sigue consignas (o terminó): siempre lo mismo.
STOP_ACTION = "Stop y Play en el TP antes de relanzar"

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

# Valores de RTDE (guía de RTDE de Universal Robots). Los motivos de parada dan el número: el TP
# muestra su nombre.
ROBOT_MODE_RUNNING = 7
SAFETY_MODES_OK = (1, 2)  # NORMAL y REDUCED: REDUCED es funcionamiento normal con límites más bajos
RUNTIME_PLAYING = 2


class FollowerStartError(RuntimeError):
    """El follower no llegó a quedar armado; el motivo dice qué falló."""


class FollowerStoppedError(RuntimeError):
    """El follower ya paró (motivo en el mensaje); hay que dar Play en el TP y relanzar el comando."""


class _ConnectionLostError(Exception):
    """Interna: al armar es un fallo de arranque, en teleoperación una parada y en `disconnect` se registra."""


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
        self._enabled, self._stop_reason = False, ""
        try:
            self._arm()
        except BaseException as exc:
            # El cliente oficial deja el socket abierto si falla la negociación.
            self._close()
            log.error("follower: no arrancó t_ns=%d %s", self._clock(), exc)
            raise
        log.info("follower: conectado y armado t_ns=%d", self._clock())

    def _arm(self) -> None:
        ip = self._config.follower.ip
        try:
            self._command = start_output(
                self._connection(), OUTPUT_NAMES, OUTPUT_TYPES, READ_HZ, inputs=(COMMAND_NAMES, COMMAND_TYPES)
            )
        except (RTDEException, OSError) as exc:
            # OSError: `sock.sendall` del cliente oficial; RTDEException: el resto de sus fallos.
            raise FollowerStartError(f"no se puede conectar por RTDE a {ip}: {exc}") from exc
        pkt = self._receive_arming()
        self._check_ready(pkt)
        # Consigna = pose actual y deshabilitado desde la primera escritura: nunca hay salto.
        self._write_disabled(pkt.actual_q)
        self._wait_for_heartbeat(pkt)

    def _write_disabled(self, q_rad: Sequence[float] | None) -> None:
        try:
            self._write(q_rad, enable=0)
        except _ConnectionLostError as exc:
            raise FollowerStartError(str(exc)) from exc

    def _wait_for_heartbeat(self, pkt: Any) -> None:
        """El programa del TP solo late ya armado, y se arma al ver un contador nuevo con enable=0.

        Se reescribe enable=0 con un contador nuevo en cada paquete: así se arma aunque el registro
        ya valiera lo de la primera escritura (otro proceso empezó con el mismo contador), y al volver
        el programa ya está armado, antes de que llegue ningún enable=1.
        """
        hb0 = pkt.output_int_register_0
        deadline_ns = self._clock() + round(ARM_TIMEOUT_S * 1e9)
        while pkt.output_int_register_0 == hb0:
            if self._clock() > deadline_ns:
                raise FollowerStartError(
                    "el programa del follower no está en marcha: dale a Play en el TP "
                    f"(heartbeat sin cambios en {ARM_TIMEOUT_S} s)"
                )
            pkt = self._receive_arming()
            self._write_disabled(None)
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

    @staticmethod
    def _check_ready(pkt: Any) -> None:
        # `runtime_state` no: el programa del TP puede arrancar mientras se espera su heartbeat.
        if pkt.robot_mode != ROBOT_MODE_RUNNING:
            raise FollowerStartError(
                f"el follower no está listo: robot_mode={pkt.robot_mode} "
                "(encender y soltar frenos desde el Teach Pendant)"
            )
        if pkt.safety_mode not in SAFETY_MODES_OK:
            raise FollowerStartError(
                f"el follower no está listo: safety_mode={pkt.safety_mode} (ver el TP)"
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
            reason = self._drain() or self._robot_fault(self._last_pkt)
        except _ConnectionLostError as exc:
            raise self._stop(str(exc)) from exc
        if reason:
            raise self._stop(reason)
        return self._last_pkt

    def _drain(self) -> str:
        """Se queda con el último paquete llegado y devuelve la primera parada vista en cualquiera.

        Se miran todos: un Stop y Play en el TP mientras LeRobot no lee deja el último en PLAYING,
        pero el programa ya no es el que seguía las consignas. La edad se mide después de vaciar:
        tras una pausa larga llegan todos los acumulados y no es una conexión perdida.
        """
        rtde = self._connection()
        newest = None
        fault = ""
        try:
            while (pkt := rtde.receive_buffered(buffer_limit=BUFFER_LIMIT)) is not None:
                newest = pkt
                fault = fault or self._robot_fault(pkt)
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
        return fault

    @staticmethod
    def _robot_fault(pkt: Any) -> str:
        if pkt.safety_mode not in SAFETY_MODES_OK:
            return f"parada de seguridad del follower: safety_mode={pkt.safety_mode} (ver el TP)"
        if pkt.robot_mode != ROBOT_MODE_RUNNING:
            return f"el follower dejó de estar en marcha: robot_mode={pkt.robot_mode} (ver el TP)"
        if pkt.runtime_state != RUNTIME_PLAYING:
            return f"el programa del follower no está en ejecución: runtime_state={pkt.runtime_state} (ver el TP)"
        return ""

    def _stop(self, reason: str) -> FollowerStoppedError:
        """La primera parada es definitiva: desde aquí todas las llamadas lanzan con su motivo."""
        reason = f"{reason} → {STOP_ACTION}"
        self._stop_reason = reason
        log.error("follower: parado t_ns=%d %s", self._clock(), reason)
        return FollowerStoppedError(reason)

    def _write(self, q_rad: Sequence[float] | None, enable: int) -> None:
        """Un paquete con consigna, enable y contador nuevo. `q_rad=None` repite la consigna anterior,
        que el objeto de la receta conserva."""
        if q_rad is not None:
            for name, value in zip(SETPOINT_NAMES, q_rad, strict=True):
                setattr(self._command, name, value)
        self._command.input_int_register_0 = enable
        self._counter += 1  # INT32: a 30 Hz tardaría más de 2 años en desbordar
        self._command.input_int_register_1 = self._counter
        # El cliente oficial devuelve None si no puede enviar, pero `sock.sendall` sí lanza
        # OSError (p. ej. conexión reiniciada).
        try:
            sent = self._connection().send(self._command)
        except OSError as exc:
            raise _ConnectionLostError(f"se perdió la conexión RTDE: {exc}") from exc
        if not sent:
            raise _ConnectionLostError("se perdió la conexión RTDE: no se pudo escribir la consigna")

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
                self._write(None, enable=0)
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
