"""Proceso de streaming del follower UR: el único sitio que escribe consignas al robot.

Corre en un proceso hijo con su propia conexión RTDE (las conexiones nunca cruzan procesos) y
habla con el padre solo a través de `SharedState`. El ritmo lo marca el robot: un ciclo por
paquete RTDE recibido, así que no hace falta temporizador propio.

Estados: ARMING (subiendo el URScript) → WAIT (quieto, sin consignas) → RUN ⇄ HOLD → STOP.
Toda salida, normal o por excepción, publica STOP con su motivo en cuanto ocurre y pasa por el
mismo cierre: enable=0 y `stop` por el Dashboard. Es una segunda capa: la seguridad real es la de PolyScope.
"""

import logging
import math
import multiprocessing
import signal
import threading
from collections import deque
from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from enum import IntEnum
from multiprocessing.context import BaseContext
from typing import Any, Protocol

import rtde.rtde as rtde
from rtde.rtde import RTDEException

from ur_core.clock import now_ns
from ur_core.config import N_JOINTS, FollowerConfig
from ur_core.dashboard import DashboardClient
from ur_core.follower_script import render_follower_script
from ur_core.motion import interpolate, limit_step
from ur_core.secondary import send_script

log = logging.getLogger("ur_core.follower")

RTDE_PORT = 30004
# Tiempo interno de arranque, no un ajuste del operador: subir el script y que empiece a latir.
ARM_TIMEOUT_S = 6.0
PERIOD_WINDOW_S = 10.0  # ventana de p50/p99 del periodo
REASON_BYTES = 256
# Un `multiprocessing.Lock` no se libera si su dueño muere con él cogido. Se tiene microsegundos:
# no conseguirlo en este tiempo significa que el otro proceso murió con él.
LOCK_TIMEOUT_S = 1.0

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
# Una sola receta de entrada: consigna y enable llegan siempre juntos al robot, en un paquete.
COMMAND_NAMES = [*SETPOINT_NAMES, "input_int_register_0"]
COMMAND_TYPES = ["DOUBLE"] * N_JOINTS + ["INT32"]

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


class FollowerState(IntEnum):
    ARMING = 0
    WAIT = 1
    RUN = 2
    HOLD = 3
    STOP = 4


class FollowerStartError(RuntimeError):
    """El follower no llegó a quedar armado (WAIT); el motivo dice qué falló."""


class FollowerStoppedError(RuntimeError):
    """El follower ya paró (motivo en el mensaje); hay que relanzar el comando desde el TP.

    Vive aquí y no en `follower.py` porque también la lanza `SharedState` en el lado del padre.
    """


class _StreamLostError(Exception):
    pass


class RtdeConnection(Protocol):
    """Lo que el bucle usa del cliente RTDE oficial (`rtde.rtde.RTDE`)."""

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def send_output_setup(self, variables: list[str], types: list[str], frequency: float) -> bool: ...
    def send_input_setup(self, variables: list[str], types: list[str]) -> Any: ...
    def send_start(self) -> bool: ...
    def send(self, input_data: Any) -> Any: ...
    def receive(self) -> Any: ...


class Dashboard(Protocol):
    def connect(self) -> None: ...
    def send(self, command: str) -> str: ...
    def close(self) -> None: ...


def _named(table: dict[int, str], value: int) -> str:
    return f"{table.get(value, '?')} ({value})"


@contextmanager
def _sigint_ignored() -> Iterator[None]:
    """Un segundo Ctrl+C durante el cierre se saltaría enable=0, el `stop` o el motivo: los
    `except Exception` no cogen KeyboardInterrupt. `signal.signal` solo vale en el hilo principal,
    y el bucle también puede correr en otro hilo (p. ej. en los tests)."""
    if threading.current_thread() is not threading.main_thread():
        yield
        return
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, previous)


class SharedState:
    """Memoria compartida padre ↔ hijo, con un único escritor por campo.

    Padre: consigna (+ secuencia + t_ns) y petición de parada. Hijo: posición medida, estado,
    motivo y estadísticas del periodo. Un solo `Lock` para todo.

    El hijo nunca espera al `Lock` en su ciclo (métodos `try_*`): con la CPU saturada (p. ej.
    codificando vídeo) el SO puede dejar al padre sin turno justo con el `Lock` cogido, y el
    bucle se quedaría decenas de ms sin escribir al robot. Si está ocupado, el hijo usa la
    consigna del ciclo anterior y publica la posición en el siguiente. Las transiciones de
    estado sí esperan: son pocas por sesión y el padre debe verlas siempre.

    Ninguna espera es indefinida (`LOCK_TIMEOUT_S`): si el otro proceso murió con el `Lock`
    cogido, el padre lanza `FollowerStoppedError` en vez de colgar LeRobot, y el hijo lo registra
    y sigue, porque siempre tiene que completar su cierre.
    """

    def __init__(self, ctx: BaseContext | None = None) -> None:
        ctx = ctx or multiprocessing.get_context("spawn")
        self._lock = ctx.Lock()
        self._target = ctx.RawArray("d", N_JOINTS)
        self._target_seq = ctx.RawValue("q", 0)
        self._target_t_ns = ctx.RawValue("q", 0)
        self._measured = ctx.RawArray("d", N_JOINTS)
        self._state = ctx.RawValue("i", FollowerState.ARMING)
        self._reason = ctx.RawArray("c", REASON_BYTES)
        self._period_stats_s = ctx.RawArray("d", 3)
        self._stop_requested = ctx.RawValue("b", 0)

    @contextmanager
    def _locked(self) -> Iterator[None]:
        """Espera del padre al `Lock`, con límite."""
        if not self._lock.acquire(timeout=LOCK_TIMEOUT_S):
            raise FollowerStoppedError(
                f"memoria compartida bloqueada más de {LOCK_TIMEOUT_S} s: "
                "el proceso del follower probablemente murió con ella cogida"
            )
        try:
            yield
        finally:
            self._lock.release()

    def write_target(self, q_rad: Sequence[float], t_ns: int) -> None:
        # Consigna, secuencia y t_ns juntos: el hijo nunca ve un t_ns nuevo con una consigna vieja.
        with self._locked():
            self._target[:] = list(q_rad)
            self._target_seq.value += 1
            self._target_t_ns.value = t_ns

    def try_read_target(self) -> tuple[int, int, list[float]] | None:
        """(secuencia, t_ns, consigna), o None si el `Lock` está ocupado.

        Secuencia 0 = todavía no ha llegado ninguna consigna.
        """
        if not self._lock.acquire(block=False):
            return None
        try:
            return self._target_seq.value, self._target_t_ns.value, list(self._target)
        finally:
            self._lock.release()

    def try_write_measured(self, q_rad: Sequence[float]) -> bool:
        if not self._lock.acquire(block=False):
            return False
        try:
            self._measured[:] = list(q_rad)
            return True
        finally:
            self._lock.release()

    def write_measured(self, q_rad: Sequence[float]) -> None:
        """Versión bloqueante de `try_write_measured`, para la pose con la que se publica WAIT."""
        if not self._child_acquire("la posición medida"):
            return
        try:
            self._measured[:] = list(q_rad)
        finally:
            self._lock.release()

    def read_measured(self) -> list[float]:
        with self._locked():
            return list(self._measured)

    def _child_acquire(self, what: str) -> bool:
        """Espera del hijo al `Lock`, con límite: si no lo consigue lo registra y sigue."""
        if self._lock.acquire(timeout=LOCK_TIMEOUT_S):
            return True
        log.error(
            "follower: no se pudo publicar %s: memoria compartida bloqueada más de %s s "
            "(¿el proceso padre murió con ella cogida?)",
            what,
            LOCK_TIMEOUT_S,
        )
        return False

    def publish_state(self, state: FollowerState, reason: str = "") -> None:
        data = reason.encode("utf-8")[: REASON_BYTES - 1]
        if not self._child_acquire(f"el estado {state.name} ({reason})"):
            return
        try:
            self._state.value = state
            self._reason.value = data
        finally:
            self._lock.release()

    def read_state(self) -> tuple[FollowerState, str]:
        with self._locked():
            return FollowerState(self._state.value), self._reason.value.decode("utf-8", errors="ignore")

    def try_write_period_stats(self, p50_s: float, p99_s: float, max_s: float) -> bool:
        if not self._lock.acquire(block=False):
            return False
        try:
            self._period_stats_s[:] = [p50_s, p99_s, max_s]
            return True
        finally:
            self._lock.release()

    def read_period_stats(self) -> tuple[float, float, float]:
        with self._locked():
            p50_s, p99_s, max_s = self._period_stats_s
            return p50_s, p99_s, max_s

    # Un único byte que solo pasa de 0 a 1: se lee y escribe sin `Lock`.
    def request_stop(self) -> None:
        self._stop_requested.value = 1

    def stop_requested(self) -> bool:
        return bool(self._stop_requested.value)


class StreamingLoop:
    """Máquina de estados del follower. Las dependencias se inyectan para probarla sin robot."""

    def __init__(
        self,
        follower: FollowerConfig,
        start_tolerance_rad: float,
        shared: SharedState,
        rtde: RtdeConnection,
        dashboard: Dashboard,
        send_script: Callable[[str, str], None],
        clock: Callable[[], int] = now_ns,
        parent_alive: Callable[[], bool] = lambda: True,
    ) -> None:
        self._cfg = follower
        self._tol_rad = start_tolerance_rad
        self._shared = shared
        self._rtde = rtde
        self._dashboard = dashboard
        self._send_script = send_script
        self._clock = clock
        self._parent_alive = parent_alive
        self._max_step_rad = follower.servo.max_joint_speed_rad_s / follower.servo.hz
        self._stop_ns = round(follower.watchdog.stop_s * 1e9)
        self._hold_ns = round(follower.watchdog.hold_s * 1e9)
        self.events: list[tuple[int, FollowerState, str]] = []  # (t_ns, estado nuevo, motivo)
        self._state = FollowerState.ARMING
        self._reason = ""
        self._command: Any = None
        self._script_sent = False
        self._armed_q_rad: list[float] = []
        self._last_cmd_rad: list[float] = []
        self._last_target_read: tuple[int, int, list[float]] = (0, 0, [])
        self._seg_seq = 0
        self._seg_start_rad: list[float] = []
        self._seg_target_rad: list[float] = []
        self._hb = 0
        self._hb_change_ns = 0
        self._last_rx_ns: int | None = None
        self._periods_ns: deque[int] = deque(maxlen=round(PERIOD_WINDOW_S * follower.servo.hz))
        self._max_period_ns = 0
        self._stats_every = max(1, round(follower.servo.hz))  # una vez por segundo
        self._cycles = 0

    # --- ciclo de vida ------------------------------------------------------------------
    def serve(self) -> None:
        """Arma el follower y sirve consignas hasta STOP. Siempre termina en STOP publicado."""
        try:
            self._arm()
            while self._state != FollowerState.STOP:
                self._cycle()
        except (FollowerStartError, _StreamLostError) as exc:
            log.error("follower: %s", exc)
            self._transition(FollowerState.STOP, str(exc))
        except KeyboardInterrupt:
            # Ctrl+C también llega al proceso hijo (misma consola en Windows, mismo grupo de
            # procesos en Linux): es una parada normal, no un error.
            log.warning("follower: interrumpido con Ctrl+C")
            self._transition(FollowerState.STOP, "interrumpido (Ctrl+C)")
        except Exception as exc:
            log.exception("follower: error inesperado")
            self._transition(FollowerState.STOP, f"error inesperado: {exc!r}")
        finally:
            with _sigint_ignored():
                self._shutdown()

    def _arm(self) -> None:
        ip = self._cfg.ip
        try:
            self._rtde.connect()
        except (RTDEException, OSError) as exc:
            # RTDEException: el puerto acepta pero el controlador no negocia el protocolo (Docker
            # publica el puerto aunque URSim aún esté arrancando).
            raise FollowerStartError(
                f"no se puede conectar por RTDE a {ip}: {exc} (¿el controlador todavía está arrancando?)"
            ) from exc
        servo_hz = self._cfg.servo.hz
        if not self._rtde_setup(self._rtde.send_output_setup, OUTPUT_NAMES, OUTPUT_TYPES, frequency=servo_hz):
            raise FollowerStartError(f"RTDE: el robot rechazó la receta de salida {OUTPUT_NAMES}")
        self._command = self._input_setup()
        if not self._rtde_setup(self._rtde.send_start):
            raise FollowerStartError("RTDE: no se pudo iniciar la sincronización")
        try:
            self._dashboard.connect()
        except OSError as exc:
            raise FollowerStartError(f"no se puede conectar al Dashboard de {ip}: {exc}") from exc

        pkt = self._receive()
        self._check_ready(pkt)
        self._armed_q_rad = list(pkt.actual_q)
        self._shared.try_write_measured(self._armed_q_rad)
        # Consigna = pose actual y deshabilitado ANTES de subir el script: nunca hay salto.
        self._write(self._armed_q_rad, enable=0)
        self._upload_script()

        hb0 = pkt.output_int_register_0
        deadline_ns = self._clock() + round(ARM_TIMEOUT_S * 1e9)
        while pkt.output_int_register_0 == hb0:
            if self._clock() > deadline_ns:
                raise FollowerStartError(
                    f"el programa del follower no arrancó: heartbeat sin cambios en {ARM_TIMEOUT_S} s"
                )
            pkt = self._receive()
            self._shared.try_write_measured(pkt.actual_q)
            self._write(self._armed_q_rad, enable=0)  # alimenta el watchdog
        self._hb = pkt.output_int_register_0
        self._hb_change_ns = self._clock()
        # Las escrituras `try_*` del armado pueden haber coincidido todas con el padre leyendo el
        # estado: `connect()` vuelve al ver WAIT y `get_joints()` ya debe dar la pose real.
        self._shared.write_measured(pkt.actual_q)
        self._transition(FollowerState.WAIT)

    def _rtde_setup(self, call: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
        """Una llamada de configuración del cliente oficial, con sus fallos como FollowerStartError.

        Envuelve solo la llamada a la librería: un AttributeError de nuestro código no es "el robot
        no respondió", es un error inesperado.
        """
        try:
            return call(*args, **kwargs)
        except AttributeError as exc:
            # El cliente oficial hace `result.types` sobre la respuesta sin comprobar que llegó:
            # si el robot no contesta a tiempo lanza AttributeError en vez de devolver un error.
            raise FollowerStartError(f"el robot {self._cfg.ip} no respondió a la configuración RTDE") from exc
        except (RTDEException, OSError) as exc:
            # `sock.sendall` lanza OSError; el cliente lanza RTDEException si el robot cierra.
            raise FollowerStartError(
                f"RTDE: se perdió la conexión con {self._cfg.ip} durante la configuración: {exc}"
            ) from exc

    def _input_setup(self) -> Any:
        busy = (
            "RTDE: registros de entrada input_double_register_0..5 / input_int_register_0 "
            "no disponibles (¿los usa otro cliente RTDE o un bus de campo?)"
        )
        try:
            command = self._rtde_setup(self._rtde.send_input_setup, COMMAND_NAMES, COMMAND_TYPES)
        except ValueError as exc:
            # El cliente oficial lanza ValueError al leer `IN_USE` en la respuesta, en vez de
            # devolver None. Solo aquí: en la receta de salida significaría otra cosa.
            raise FollowerStartError(f"{busy}: {exc}") from exc
        if command is None:  # tipos distintos a los pedidos
            raise FollowerStartError(busy)
        return command

    def _upload_script(self) -> None:
        script = render_follower_script(self._cfg.servo, self._cfg.watchdog)
        self._script_sent = True  # a partir de aquí puede haber un programa nuestro corriendo
        try:
            self._send_script(self._cfg.ip, script)
        except OSError as exc:
            raise FollowerStartError(
                f"no se pudo subir el URScript por la interfaz secundaria: {exc}"
            ) from exc

    def _check_ready(self, pkt: Any) -> None:
        if pkt.robot_mode != ROBOT_MODE_RUNNING:
            raise FollowerStartError(
                f"el follower no está listo: robot_mode={_named(ROBOT_MODES, pkt.robot_mode)} "
                "(encender y soltar frenos desde el Teach Pendant)"
            )
        if pkt.safety_mode not in SAFETY_MODES_OK:
            raise FollowerStartError(
                f"el follower no está listo: safety_mode={_named(SAFETY_MODES, pkt.safety_mode)}"
            )

    def _shutdown(self) -> None:
        # Sin ningún comando escrito no hay nada habilitado (y el cliente oficial no empaqueta una
        # receta con campos sin valor). Si lo hay, se repite el último con enable=0: la receta
        # lleva siempre la consigna, y repetir la última no pide ningún movimiento nuevo.
        if self._last_cmd_rad and self._rtde.is_connected():
            try:
                self._write(self._last_cmd_rad, enable=0)
            except Exception:
                log.exception("follower: no se pudo escribir enable=0 (el watchdog del robot lo parará)")
                self._reason += " (enable=0 no se pudo escribir; lo para el watchdog del robot)"
        if self._script_sent:
            try:
                self._backup_stop()
            except Exception:
                log.exception("follower: el `stop` de respaldo por el Dashboard falló")
        try:
            self._dashboard.close()
        except Exception:
            log.exception("follower: error al cerrar el Dashboard")
        # Dentro del `finally` de `serve`: una excepción aquí sustituiría a la original.
        try:
            self._publish_period_stats()
        except Exception:
            log.exception("follower: no se pudieron publicar las estadísticas del periodo")
        self._shared.publish_state(FollowerState.STOP, self._reason)  # por si el cierre lo amplió
        try:
            self._rtde.disconnect()
        except Exception:
            log.exception("follower: error al cerrar la conexión RTDE")

    def _backup_stop(self) -> None:
        # Conexión nueva: la del armado puede llevar muerta desde entonces, y `connect()` no la
        # renueva mientras el socket siga abierto.
        self._dashboard.close()
        self._dashboard.connect()
        answer = self._dashboard.send("stop")
        if not answer.startswith("Stopped"):  # lo que contesta el Dashboard de UR si lo paró
            log.error("follower: el Dashboard no confirmó el `stop` de respaldo: %r", answer)

    # --- un ciclo por paquete -----------------------------------------------------------
    def _cycle(self) -> None:
        pkt = self._receive()
        now_ns = self._clock()
        self._record_period(now_ns)
        self._shared.try_write_measured(pkt.actual_q)
        reason = self._robot_fault(pkt, now_ns) or self._external_stop()
        if reason:
            self._transition(FollowerState.STOP, reason)
            return
        latest = self._shared.try_read_target()
        if latest is not None:
            self._last_target_read = latest
        seq, t_ns, target_rad = self._last_target_read
        if self._state == FollowerState.WAIT:
            self._wait_cycle(pkt, seq, t_ns, target_rad, now_ns)
        else:
            self._run_cycle(seq, t_ns, target_rad, now_ns)

    def _wait_cycle(self, pkt: Any, seq: int, t_ns: int, target_rad: list[float], now_ns: int) -> None:
        if seq == 0:
            self._write(self._armed_q_rad, enable=0)
            return
        error_rad = max(abs(t - m) for t, m in zip(target_rad, pkt.actual_q, strict=True))
        if error_rad > self._tol_rad:
            self._transition(
                FollowerState.STOP,
                f"primera consigna lejos de la posición actual "
                f"({math.degrees(error_rad):.1f}° > {math.degrees(self._tol_rad):.1f}°)",
            )
            return
        self._transition(FollowerState.RUN)
        self._run_cycle(seq, t_ns, target_rad, now_ns)

    def _run_cycle(self, seq: int, t_ns: int, target_rad: list[float], now_ns: int) -> None:
        if seq != self._seg_seq:
            # Parte del último comando escrito, no de la consigna anterior: la salida es continua
            # aunque la consigna nueva llegue antes de terminar la anterior.
            self._seg_seq = seq
            self._seg_start_rad, self._seg_target_rad = list(self._last_cmd_rad), target_rad
            if self._state == FollowerState.HOLD:
                self._transition(FollowerState.RUN)
        age_ns = now_ns - t_ns
        if age_ns > self._stop_ns:
            self._transition(FollowerState.STOP, f"sin consignas nuevas durante {age_ns / 1e6:.0f} ms")
            return
        if age_ns > self._hold_ns:
            if self._state == FollowerState.RUN:
                self._transition(FollowerState.HOLD, f"sin consignas nuevas durante {age_ns / 1e6:.0f} ms")
            self._write(self._last_cmd_rad, enable=1)
            return
        interpolated = interpolate(
            self._seg_start_rad, self._seg_target_rad, age_ns / 1e9, self._cfg.servo.target_period_s
        )
        self._write(limit_step(interpolated, self._last_cmd_rad, self._max_step_rad), enable=1)

    def _robot_fault(self, pkt: Any, now_ns: int) -> str:
        if pkt.safety_mode not in SAFETY_MODES_OK:
            return f"parada de seguridad del follower: safety_mode={_named(SAFETY_MODES, pkt.safety_mode)}"
        if pkt.robot_mode != ROBOT_MODE_RUNNING:
            return f"el follower dejó de estar en marcha: robot_mode={_named(ROBOT_MODES, pkt.robot_mode)}"
        if pkt.runtime_state != RUNTIME_PLAYING:
            return (
                "el programa del follower no está en ejecución: "
                f"runtime_state={_named(RUNTIME_STATES, pkt.runtime_state)}"
            )
        if pkt.output_int_register_0 != self._hb:
            self._hb, self._hb_change_ns = pkt.output_int_register_0, now_ns
        elif now_ns - self._hb_change_ns > self._stop_ns:
            return f"el programa del follower no responde: heartbeat parado {self._cfg.watchdog.stop_s} s"
        return ""

    def _external_stop(self) -> str:
        if self._shared.stop_requested():
            return "parada pedida (disconnect)"
        if not self._parent_alive():
            return "el proceso padre ha terminado"
        return ""

    # --- E/S ----------------------------------------------------------------------------
    def _receive(self) -> Any:
        try:
            pkt = self._rtde.receive()
        except (RTDEException, OSError) as exc:
            raise _StreamLostError(f"se perdió el stream RTDE: {exc}") from exc
        if pkt is None:
            raise _StreamLostError("se perdió el stream RTDE: sin datos del robot")
        return pkt

    def _write(self, q_rad: Sequence[float], enable: int) -> None:
        for i, value in enumerate(q_rad):
            setattr(self._command, SETPOINT_NAMES[i], value)
        self._command.input_int_register_0 = enable
        # El cliente oficial devuelve False/None si no puede enviar, pero `sock.sendall` sí lanza
        # OSError (p. ej. conexión reiniciada).
        try:
            sent = self._rtde.send(self._command)
        except OSError as exc:
            raise _StreamLostError(f"se perdió el stream RTDE: {exc}") from exc
        if not sent:
            raise _StreamLostError("se perdió el stream RTDE: no se pudo escribir la consigna")
        self._last_cmd_rad = list(q_rad)

    # --- observabilidad -----------------------------------------------------------------
    def _transition(self, state: FollowerState, reason: str = "") -> None:
        if self._state == FollowerState.STOP:
            return  # el primer motivo de parada es el que cuenta
        t_ns = self._clock()
        log.info("follower %s -> %s t_ns=%d %s", self._state.name, state.name, t_ns, reason)
        self.events.append((t_ns, state, reason))
        self._state, self._reason = state, reason
        # También STOP, sin esperar al cierre (puede tardar segundos): hasta verlo, el padre
        # seguiría aceptando consignas y dando una pose que ya no se actualiza.
        self._shared.publish_state(state, reason)

    def _record_period(self, now_ns: int) -> None:
        if self._last_rx_ns is not None:
            period_ns = now_ns - self._last_rx_ns
            self._periods_ns.append(period_ns)
            self._max_period_ns = max(self._max_period_ns, period_ns)
            # Por contador y no por len(): con la ventana llena len() no cambia y se publicaría
            # (ordenando la ventana) en cada ciclo.
            self._cycles += 1
            if self._cycles % self._stats_every == 0:
                self._publish_period_stats()
        self._last_rx_ns = now_ns

    def _publish_period_stats(self) -> None:
        if not self._periods_ns:
            return
        ordered = sorted(self._periods_ns)
        p50_ns = ordered[len(ordered) // 2]
        p99_ns = ordered[min(len(ordered) - 1, int(0.99 * len(ordered)))]
        self._shared.try_write_period_stats(p50_ns / 1e9, p99_ns / 1e9, self._max_period_ns / 1e9)


def streaming_main(follower: FollowerConfig, start_tolerance_rad: float, shared: SharedState) -> None:
    """Punto de entrada del proceso hijo: crea aquí sus conexiones reales y sirve hasta STOP."""
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s [%(name)s] %(message)s")
    parent = multiprocessing.parent_process()
    loop = StreamingLoop(
        follower,
        start_tolerance_rad,
        shared,
        rtde=rtde.RTDE(follower.ip, RTDE_PORT),
        dashboard=DashboardClient(follower.ip),
        send_script=send_script,
        parent_alive=parent.is_alive if parent is not None else lambda: True,
    )
    loop.serve()
