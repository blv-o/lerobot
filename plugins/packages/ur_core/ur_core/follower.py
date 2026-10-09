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
from multiprocessing.util import Finalize
from typing import Protocol

import rtde.rtde as rtde

from ur_core.clock import now_ns
from ur_core.config import N_JOINTS, UR_TYPES, FollowerConfig, TeleopConfig
from ur_core.dashboard import DASHBOARD_TIMEOUT_S
from ur_core.startup import check_pose, start_pose_text
from ur_core.streaming import (
    ARM_TIMEOUT_S,
    LOCK_TIMEOUT_S,
    RTDE_PORT,
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
# Peor cierre del proceso de streaming una vez decidida la parada: el `receive` en curso y el `send`
# de enable=0 (hasta `rtde.DEFAULT_TIMEOUT` cada uno en el cliente oficial), el `stop` por el
# Dashboard (conexión, bienvenida y respuesta: hasta `DASHBOARD_TIMEOUT_S` cada una; enviar una
# línea no espera) y las dos publicaciones de STOP (hasta `LOCK_TIMEOUT_S` cada una). Terminarlo
# antes deja el cierre a medias: sin enable=0 ni `stop`, solo queda el watchdog del robot.
SHUTDOWN_JOIN_S = 2 * rtde.DEFAULT_TIMEOUT + 3 * DASHBOARD_TIMEOUT_S + 2 * LOCK_TIMEOUT_S + JOIN_MARGIN_S
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
    ) -> None:
        self._config = config
        self._launch = launch
        self._rtde_factory = rtde_factory
        self._shared: SharedState | None = None
        self._worker: Worker | None = None
        self._exit_disconnect: Finalize | None = None

    @property
    def is_connected(self) -> bool:
        return self._worker is not None

    # --- arranque -----------------------------------------------------------------------
    def start_prompt(self) -> str:
        follower = self._config.follower
        pose = start_pose_text(self._config.start_pose_rad, self._config.start_tolerance_rad)
        return (
            f"Follower {follower.type} ({follower.ip}), desde el Teach Pendant:\n"
            "  - Programa del follower cargado y en marcha (Play).\n"
            f"  - Robot en la posición inicial {pose} (base..wrist_3).\n"
            "¿Continuar? (yes/no): "
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
            follower.servo.hz,  # leído aquí: un AttributeError nuestro no es "el robot no respondió"
            self._config.start_pose_rad,
            self._config.start_tolerance_rad,
        )

    def connect(self) -> None:
        """Lanza el proceso de streaming y vuelve cuando el follower está armado y quieto (WAIT)."""
        if self._worker is not None:
            return
        shared = SharedState()
        worker = self._launch(self._config.follower, self._config.start_tolerance_rad, shared)
        self._shared, self._worker = shared, worker
        # Al salir del intérprete sin disconnect() (un `finally` que lanzó antes de llegar a él),
        # multiprocessing termina el proceso daemon sin su cierre: sin enable=0 ni `stop`, el
        # robot lo para su watchdog y queda en parada de protección. Un Finalize con prioridad
        # >= 0 corre dentro de su hook de salida justo antes de terminar los daemon, sea cual
        # sea el orden de los hooks de atexit (que `multiprocessing.get_logger()` reordena).
        self._exit_disconnect = Finalize(self, self.disconnect, exitpriority=0)
        deadline_ns = now_ns() + round((ARM_TIMEOUT_S + CONNECT_MARGIN_S) * 1e9)
        while True:
            alive = worker.is_alive()  # antes de leer el estado: si murió, el STOP ya está publicado
            try:
                state, reason = shared.read_state()
            except FollowerStoppedError as exc:  # murió con el Lock cogido
                self._abort(JOIN_MARGIN_S)
                raise FollowerStartError(str(exc)) from exc
            if state == FollowerState.WAIT:
                return
            if state == FollowerState.STOP:
                # STOP se publica antes del cierre: si ya se vio el programa del TP, falta su `stop`.
                self._abort(SHUTDOWN_JOIN_S)
                raise FollowerStartError(reason)
            if not alive:
                self._abort(JOIN_MARGIN_S)
                raise FollowerStartError("el proceso del follower terminó sin armar ni publicar el motivo")
            if now_ns() > deadline_ns:
                self._abort(JOIN_MARGIN_S)  # atascado: su propio ARM_TIMEOUT_S ya pasó
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

        Tras STOP o con el proceso de streaming muerto lanza en vez de devolver la última pose: ya
        no se actualiza, y una pose congelada haría que LeRobot siguiera grabando con el robot parado.
        """
        shared, worker = self._connected()
        self._raise_if_stopped(shared, alive=worker.is_alive())
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
        self._worker.join(timeout=SHUTDOWN_JOIN_S)
        if self._worker.is_alive():
            log.error(
                "follower: el proceso de streaming no terminó a tiempo; se fuerza (el watchdog del robot lo para)"
            )
            self._worker.terminate()
            self._worker.join(timeout=JOIN_MARGIN_S)
        self._worker = None
        self._cancel_exit_disconnect()

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

    def _abort(self, join_s: float) -> None:
        assert self._worker is not None
        self._worker.join(timeout=join_s)
        if self._worker.is_alive():
            self._worker.terminate()
            self._worker.join(timeout=JOIN_MARGIN_S)
        self._worker = None
        self._cancel_exit_disconnect()

    def _cancel_exit_disconnect(self) -> None:
        # El registro de multiprocessing guarda `self.disconnect`: sin cancelarlo, retiene el objeto.
        if self._exit_disconnect is not None:
            self._exit_disconnect.cancel()
            self._exit_disconnect = None


FOLLOWERS: dict[str, type[UrFollowerCore]] = dict.fromkeys(UR_TYPES, UrFollowerCore)
