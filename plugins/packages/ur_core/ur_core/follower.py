"""Follower UR visto desde el proceso de LeRobot: lanza el proceso de streaming y le pasa consignas.

Esta clase nunca habla con el robot durante la teleoperación: solo lee y escribe `SharedState`,
así `send_joints` no bloquea el bucle de LeRobot aunque el robot o la red vayan lentos. La única
conexión que abre es la breve de `check_start`, antes de lanzar el proceso.
"""

import logging
import math
import multiprocessing
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Protocol

import rtde.rtde as rtde
from rtde.rtde import RTDEException

from ur_core.clock import now_ns
from ur_core.config import N_JOINTS, UR_TYPES, FollowerConfig, TeleopConfig
from ur_core.dashboard import DashboardClient
from ur_core.startup import pose_errors
from ur_core.streaming import (
    ARM_TIMEOUT_S,
    RTDE_PORT,
    Dashboard,
    FollowerStartError,
    FollowerState,
    FollowerStoppedError,
    RtdeConnection,
    SharedState,
    streaming_main,
)

log = logging.getLogger("ur_core.follower")

# Además de ARM_TIMEOUT_S: conexión RTDE + Dashboard y arranque del proceso spawn (importa todo).
CONNECT_MARGIN_S = 15.0
JOIN_MARGIN_S = 2.0
POLL_S = 0.01


@dataclass(frozen=True)
class FollowerStatus:
    state: FollowerState
    stop_reason: str
    period_p50_s: float
    period_p99_s: float
    period_max_s: float


class Worker(Protocol):
    """Lo que se usa de `multiprocessing.Process` (o de un hilo equivalente en los tests)."""

    def is_alive(self) -> bool: ...
    def join(self, timeout: float | None = None) -> None: ...
    def terminate(self) -> None: ...


Launcher = Callable[[FollowerConfig, float, SharedState], Worker]


def spawn_worker(follower: FollowerConfig, start_tolerance_rad: float, shared: SharedState) -> Worker:
    # spawn en todas las plataformas: el mismo comportamiento en Windows y Linux.
    process = multiprocessing.get_context("spawn").Process(
        target=streaming_main,
        args=(follower, start_tolerance_rad, shared),
        name="ur-follower-streaming",
        daemon=True,
    )
    process.start()
    return process


def _rtde_connection(ip: str) -> RtdeConnection:
    return rtde.RTDE(ip, RTDE_PORT)


class UrFollowerCore:
    def __init__(
        self,
        config: TeleopConfig,
        launch: Launcher = spawn_worker,
        rtde_factory: Callable[[str], RtdeConnection] = _rtde_connection,
        dashboard_factory: Callable[[str], Dashboard] = DashboardClient,
    ) -> None:
        self._config = config
        self._launch = launch
        self._rtde_factory = rtde_factory
        self._dashboard_factory = dashboard_factory
        self._shared: SharedState | None = None
        self._worker: Worker | None = None

    @property
    def is_connected(self) -> bool:
        return self._worker is not None

    # --- arranque -----------------------------------------------------------------------
    def start_prompt(self) -> str:
        follower = self._config.follower
        pose = ", ".join(f"{round(math.degrees(q), 1):g}" for q in self._config.start_pose_rad)
        tolerance = f"{round(math.degrees(self._config.start_tolerance_rad), 1):g}"
        return (
            f"Follower {follower.type} ({follower.ip}), desde el Teach Pendant:\n"
            "  - Remote Control activado y modo Remote seleccionado.\n"
            f"  - Robot en la posición inicial [{pose}]° ± {tolerance}° (base..wrist_3).\n"
            "¿Continuar? (yes/no): "
        )

    def check_start(self) -> list[str]:
        """Problemas que impiden arrancar ([] = listo). No lanza: cada fallo es una línea."""
        ip = self._config.follower.ip
        errors: list[str] = []
        try:
            answer = self._ask_dashboard("is in remote control")
            if answer.strip().lower() != "true":
                errors.append(f"follower: no está en Remote Control (el Dashboard responde {answer!r})")
        except OSError as exc:
            errors.append(f"follower: no se puede conectar al Dashboard de {ip}: {exc}")
        try:
            q_rad = self._read_pose()
            errors += pose_errors(
                "follower", q_rad, self._config.start_pose_rad, self._config.start_tolerance_rad
            )
        except (OSError, RTDEException) as exc:
            errors.append(f"follower: no se puede leer la posición por RTDE en {ip}: {exc}")
        return errors

    def _ask_dashboard(self, command: str) -> str:
        dashboard = self._dashboard_factory(self._config.follower.ip)
        dashboard.connect()
        try:
            return dashboard.send(command)
        finally:
            dashboard.close()

    def _read_pose(self) -> list[float]:
        con = self._rtde_factory(self._config.follower.ip)
        try:
            # Dentro del `try`: el cliente oficial abre el socket antes de negociar el protocolo,
            # y si la negociación falla lo deja abierto. Su `disconnect()` es seguro sin socket.
            con.connect()
            try:
                output_ok = con.send_output_setup(
                    ["actual_q"], ["VECTOR6D"], frequency=self._config.follower.servo.hz
                )
            except AttributeError as exc:
                # El cliente oficial hace `result.types` sobre la respuesta sin comprobar que llegó.
                raise RTDEException("el robot no respondió a la configuración RTDE") from exc
            if not output_ok:
                raise RTDEException("el robot rechazó la receta de salida actual_q")
            if not con.send_start():
                raise RTDEException("no se pudo iniciar la sincronización")
            pkt = con.receive()
            if pkt is None:
                raise RTDEException("sin datos del robot")
            return list(pkt.actual_q)
        finally:
            con.disconnect()

    def connect(self) -> None:
        """Lanza el proceso de streaming y vuelve cuando el follower está armado y quieto (WAIT)."""
        if self._worker is not None:
            return
        shared = SharedState()
        worker = self._launch(self._config.follower, self._config.start_tolerance_rad, shared)
        self._shared, self._worker = shared, worker
        deadline_ns = now_ns() + round((ARM_TIMEOUT_S + CONNECT_MARGIN_S) * 1e9)
        while True:
            alive = worker.is_alive()  # antes de leer el estado: si murió, el STOP ya está publicado
            try:
                state, reason = shared.read_state()
            except FollowerStoppedError as exc:  # murió con el Lock cogido
                self._abort()
                raise FollowerStartError(str(exc)) from exc
            if state == FollowerState.WAIT:
                return
            if state == FollowerState.STOP:
                self._abort()
                raise FollowerStartError(reason)
            if not alive:
                self._abort()
                raise FollowerStartError("el proceso del follower terminó sin armar ni publicar el motivo")
            if now_ns() > deadline_ns:
                self._abort()
                raise FollowerStartError(
                    f"el follower no quedó armado en {ARM_TIMEOUT_S + CONNECT_MARGIN_S} s; proceso terminado"
                )
            time.sleep(POLL_S)

    # --- teleoperación ------------------------------------------------------------------
    def send_joints(self, q_rad: Sequence[float]) -> None:
        """Deja la consigna (6 rad) para el proceso de streaming; no espera al robot."""
        target_rad = [float(x) for x in q_rad]
        if len(target_rad) != N_JOINTS:
            raise ValueError(f"se esperaban {N_JOINTS} articulaciones y llegan {len(target_rad)}")
        if not all(math.isfinite(x) for x in target_rad):
            raise ValueError(f"consigna con valores no finitos: {target_rad}")
        shared, worker = self._connected()
        self._raise_if_stopped(shared, alive=worker.is_alive())
        # El mismo reloj que usa el proceso de streaming para la edad de la consigna.
        shared.write_target(target_rad, now_ns())

    def get_joints(self) -> list[float]:
        """Última `actual_q` medida, en rad.

        Con el proceso de streaming muerto lanza en vez de devolver la última pose: una pose
        congelada haría que LeRobot siguiera grabando con el robot parado.
        """
        shared, worker = self._connected()
        alive = worker.is_alive()
        if not alive:
            self._raise_if_stopped(shared, alive)
        return shared.read_measured()

    def status(self) -> FollowerStatus:
        if self._shared is None:
            raise RuntimeError("follower no conectado: llama a connect()")
        state, reason = self._shared.read_state()
        p50_s, p99_s, max_s = self._shared.read_period_stats()
        return FollowerStatus(state, reason, p50_s, p99_s, max_s)

    def disconnect(self) -> None:
        """Pide la parada y espera a que el proceso pase por su cierre. Idempotente."""
        if self._worker is None or self._shared is None:
            return
        self._shared.request_stop()
        self._worker.join(timeout=self._config.follower.watchdog.stop_s + JOIN_MARGIN_S)
        if self._worker.is_alive():
            log.error(
                "follower: el proceso de streaming no terminó a tiempo; se fuerza (el watchdog del robot lo para)"
            )
            self._worker.terminate()
            self._worker.join(timeout=JOIN_MARGIN_S)
        self._worker = None

    def _connected(self) -> tuple[SharedState, Worker]:
        if self._worker is None or self._shared is None:
            raise RuntimeError("follower no conectado: llama a connect()")
        return self._shared, self._worker

    @staticmethod
    def _raise_if_stopped(shared: SharedState, alive: bool) -> None:
        """`alive` se lee ANTES que el estado: si el proceso murió después de publicar STOP, se
        ve su motivo; si murió sin publicarlo (EDR, `terminate()`, fallo nativo), el estado se
        quedó en RUN/HOLD y solo `alive` lo delata."""
        state, reason = shared.read_state()
        if state == FollowerState.STOP:
            raise FollowerStoppedError(f"el follower está parado: {reason}")
        if not alive:
            raise FollowerStoppedError(
                "el proceso del follower terminó sin publicar el motivo; el watchdog del robot lo para"
            )

    def _abort(self) -> None:
        assert self._worker is not None
        self._worker.join(timeout=JOIN_MARGIN_S)
        if self._worker.is_alive():
            self._worker.terminate()
            self._worker.join(timeout=JOIN_MARGIN_S)
        self._worker = None


FOLLOWERS: dict[str, type[UrFollowerCore]] = dict.fromkeys(UR_TYPES, UrFollowerCore)
