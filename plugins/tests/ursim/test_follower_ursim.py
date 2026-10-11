"""Follower contra el URSim de plugins/ursim (127.0.0.2), preparado a mano desde PolyScope.

Antes de lanzarlos (ver plugins/ursim/README.md): robot encendido con frenos sueltos, en modo
Remote, con el programa del follower en su carpeta de programas y en la posición inicial de
plugins/configs/ur_config.yaml. El modo Remote solo lo necesitan estos tests: antes de cada uno,
`follower_program_playing` pone el programa en marcha por el Dashboard, como haría el operador
con Play en el TP.

Con `UR_RECORD_TRACES=1` además se graban en plugins/tests/traces/ las trazas RTDE del
follower que usan los tests de replay (sin esa variable no se escribe nada).
"""

import json
import math
import multiprocessing.process
import os
import subprocess
import sys
import threading
import time
from collections.abc import Iterator
from typing import Any

import pytest
import rtde.rtde as rtde
from conftest import (
    CONFIG,
    CONFIG_PATH,
    FOLLOWER_IP,
    PLUGINS,
    dashboard,
    program_running,
    protective_stop_appears,
    release_protective_stop,
    wait_until,
)
from ur_core import FollowerState, FollowerStoppedError, UrFollowerCore
from ur_core.clock import now_ns
from ur_core.config import FollowerConfig
from ur_core.follower import Worker, spawn_worker
from ur_core.streaming import OUTPUT_NAMES, OUTPUT_TYPES, RTDE_PORT, SharedState

TRACES = PLUGINS / "tests" / "traces"
SINE_AMPLITUDE_RAD = 0.2
SINE_FREQUENCY_HZ = 0.2
SEND_HZ = 30
SINE_DURATION_S = 2 / SINE_FREQUENCY_HZ  # dos periodos completos
SHAPE_RMS_LIMIT_RAD = 0.02
DELAY_LIMIT_S = 0.2
SETTLE_S = 0.5
MAX_SEARCHED_DELAY_MS = 300  # por encima del límite, para que un retraso excesivo se vea como tal

pytestmark = [pytest.mark.ursim, pytest.mark.usefixtures("follower_program_playing")]


class TraceRecorder:
    """Graba el stream RTDE del follower con una conexión de solo lectura propia."""

    def __init__(self) -> None:
        self.packets: list[dict[str, Any]] = []
        self._stop = threading.Event()
        self._con = rtde.RTDE(FOLLOWER_IP, RTDE_PORT)
        self._con.connect()
        assert self._con.send_output_setup(OUTPUT_NAMES, OUTPUT_TYPES, frequency=CONFIG.follower.servo.hz)
        assert self._con.send_start()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            pkt = self._con.receive()
            if pkt is not None:
                self.packets.append({name: _plain(getattr(pkt, name)) for name in OUTPUT_NAMES})

    def save(self, name: str) -> None:
        self._stop.set()
        self._thread.join(timeout=5)
        self._con.disconnect()
        if os.environ.get("UR_RECORD_TRACES") == "1":
            TRACES.mkdir(exist_ok=True)
            (TRACES / f"{name}.json").write_text(json.dumps(self.packets), encoding="utf-8")


def _plain(value: Any) -> Any:
    return list(value) if isinstance(value, list | tuple) else value


@pytest.fixture
def core() -> Iterator[UrFollowerCore]:
    core = UrFollowerCore(CONFIG)
    yield core
    core.disconnect()


def test_follower_is_ready_for_teleop() -> None:
    """Posición inicial leída por RTDE: lo que comprueba el arranque."""
    assert UrFollowerCore(CONFIG).check_start() == []


def test_rtde_mode_values_match_the_ones_the_loop_expects() -> None:
    """robot_mode RUNNING = 7 y safety_mode NORMAL = 1 con los tipos de la receta del bucle."""
    con = rtde.RTDE(FOLLOWER_IP, RTDE_PORT)
    con.connect()
    try:
        assert con.send_output_setup(OUTPUT_NAMES, OUTPUT_TYPES, frequency=CONFIG.follower.servo.hz)
        assert con.send_start()
        pkt = con.receive()
    finally:
        con.disconnect()
    assert dashboard("robotmode") == "Robotmode: RUNNING" and pkt.robot_mode == 7
    assert dashboard("safetystatus") == "Safetystatus: NORMAL" and pkt.safety_mode == 1


def test_armed_follower_stays_still_in_wait_with_program_playing(core: UrFollowerCore) -> None:
    """Si runtime_state no fuera PLAYING (= 2) con el programa del TP corriendo, el bucle pararía."""
    core.connect()
    q0 = core.get_joints()
    assert program_running()
    time.sleep(2.0)
    assert core.status().state == FollowerState.WAIT
    assert max(abs(a - b) for a, b in zip(core.get_joints(), q0, strict=True)) < 1e-3


def sine_offset_rad(t_s: float) -> float:
    return SINE_AMPLITUDE_RAD * math.sin(2 * math.pi * SINE_FREQUENCY_HZ * t_s)


def shape_rms_rad(samples: list[tuple[float, list[float]]], q0: list[float], delay_s: float) -> float:
    """RMS entre lo medido y la consigna retrasada `delay_s` (todas las articulaciones)."""
    squared = [
        (m - (q + sine_offset_rad(t_s - delay_s))) ** 2
        for t_s, measured in samples
        if t_s > delay_s
        for m, q in zip(measured, q0, strict=True)
    ]
    return math.sqrt(sum(squared) / len(squared))


def test_sine_is_tracked_with_small_error_and_no_protective_stop(core: UrFollowerCore) -> None:
    """Seno de 0,2 rad a 0,2 Hz en las 6 articulaciones, enviado a 30 Hz.

    Forma y retraso se miden por separado: comparar "pedido ahora" con "medido ahora" mide
    sobre todo el retraso (lookahead del servoj + interpolación), no si el robot sigue bien.
    Forma: RMS < 0,02 rad con el retraso constante que mejor encaja. Retraso: < 200 ms.
    """
    recorder = TraceRecorder()
    core.connect()
    q0 = core.get_joints()
    period_ns = round(1e9 / SEND_HZ)
    t0_ns = now_ns()
    next_ns = t0_ns
    samples: list[tuple[float, list[float]]] = []  # (t_s, posición medida)
    while (t_now_ns := now_ns()) - t0_ns < SINE_DURATION_S * 1e9:
        t_s = (t_now_ns - t0_ns) / 1e9
        core.send_joints([q + sine_offset_rad(t_s) for q in q0])
        samples.append((t_s, core.get_joints()))
        next_ns += period_ns
        time.sleep(max(0.0, (next_ns - now_ns()) / 1e9))
    # El robot va retrasado: sin esperar a que vuelva a q0, cada ejecución lo dejaría ~2° más
    # lejos de la posición inicial y el arranque de los demás tests fallaría.
    for _ in range(round(SETTLE_S * SEND_HZ)):
        core.send_joints(q0)
        time.sleep(1 / SEND_HZ)
    status = core.status()
    core.disconnect()
    recorder.save("follower_sine")
    delays_s = [ms / 1000 for ms in range(0, MAX_SEARCHED_DELAY_MS + 1, 5)]
    delay_s = min(delays_s, key=lambda d: shape_rms_rad(samples, q0, d))
    rms_rad = shape_rms_rad(samples, q0, delay_s)
    print(
        f"forma RMS = {rms_rad:.4f} rad con retraso {delay_s * 1e3:.0f} ms "
        f"(sin descontarlo: {shape_rms_rad(samples, q0, 0.0):.4f} rad); periodo p50/p99/máx = "
        f"{status.period_p50_s * 1e3:.2f}/{status.period_p99_s * 1e3:.2f}/{status.period_max_s * 1e3:.2f} ms"
    )
    assert status.state != FollowerState.STOP, status.stop_reason
    assert "parada pedida" in core.status().stop_reason
    assert dashboard("safetystatus") == "Safetystatus: NORMAL"
    assert rms_rad < SHAPE_RMS_LIMIT_RAD
    assert delay_s < DELAY_LIMIT_S


def test_disconnect_stops_the_program_on_the_robot(core: UrFollowerCore) -> None:
    core.connect()
    core.send_joints(core.get_joints())
    wait_until(lambda: core.status().state == FollowerState.RUN, 2.0, "RUN")
    core.disconnect()
    wait_until(lambda: not program_running(), 2.0, "programa parado")


def test_program_stopped_from_pendant_stops_the_follower(core: UrFollowerCore) -> None:
    """El `stop` del Dashboard desde otra conexión hace lo mismo que el botón de parar del TP."""
    recorder = TraceRecorder()
    core.connect()
    q0 = core.get_joints()
    core.send_joints(q0)
    wait_until(lambda: core.status().state == FollowerState.RUN, 2.0, "RUN")
    stop_ns = now_ns()
    dashboard("stop")
    # LeRobot sigue mandando consignas y se entera de la parada porque send_joints lanza.
    while True:
        try:
            core.send_joints(q0)
        except FollowerStoppedError:
            break
        assert now_ns() - stop_ns < CONFIG.follower.watchdog.stop_s * 1e9 + 0.5e9
        time.sleep(1 / SEND_HZ)
    recorder.save("follower_stopped_from_pendant")
    reason = core.status().stop_reason
    assert "runtime_state" in reason or "heartbeat" in reason, reason


PARENT_SCRIPT = """
import sys, time
from ur_core import UrFollowerCore, load_config
core = UrFollowerCore(load_config(sys.argv[1]))
core.connect()
q0 = core.get_joints()
print("ready", flush=True)
while True:
    core.send_joints(q0)
    time.sleep(1 / 30)
"""


def test_killed_parent_process_stops_the_robot_within_stop_time() -> None:
    """Cierre brusco (proceso padre matado, sin `finally`): el hijo lo detecta y para el robot."""
    parent = subprocess.Popen(
        [sys.executable, "-c", PARENT_SCRIPT, str(CONFIG_PATH)],
        stdout=subprocess.PIPE,
        text=True,
        env={**os.environ},
    )
    try:
        assert parent.stdout is not None and parent.stdout.readline().strip() == "ready"
        wait_until(program_running, 2.0, "programa en marcha")
    finally:
        parent.kill()
        parent.wait(timeout=10)
    killed_ns = now_ns()
    wait_until(lambda: not program_running(), 5.0, "programa parado")
    stopped_after_s = (now_ns() - killed_ns) / 1e9
    print(f"parado {stopped_after_s * 1e3:.0f} ms después de matar al padre")
    assert stopped_after_s <= CONFIG.follower.watchdog.stop_s


EXIT_STREAMING_S = 1.0

EXITING_PARENT_SCRIPT = """
import sys, time
from ur_core import UrFollowerCore, load_config
from ur_core.clock import now_ns
core = UrFollowerCore(load_config(sys.argv[1]))
core.connect()
q0 = core.get_joints()
print("ready", flush=True)
end_ns = now_ns() + round(float(sys.argv[2]) * 1e9)
while now_ns() < end_ns:
    core.send_joints(q0)
    time.sleep(1 / 30)
print("exiting", flush=True)
sys.exit(0)  # salida normal del intérprete sin disconnect()
"""


def test_parent_exiting_without_disconnect_stops_the_robot_in_order() -> None:
    """Salida normal del padre sin `disconnect()` (p. ej. un `finally` que lanzó antes de llegar a
    él): el cierre al salir del intérprete tiene que pasar por la parada ordenada del hijo, no
    terminarlo y dejar que lo pare el watchdog del robot (que lo deja en parada de protección)."""
    assert dashboard("safetystatus") == "Safetystatus: NORMAL"
    stop_s = CONFIG.follower.watchdog.stop_s
    parent = subprocess.Popen(
        [sys.executable, "-c", EXITING_PARENT_SCRIPT, str(CONFIG_PATH), str(EXIT_STREAMING_S)],
        stdout=subprocess.PIPE,
        text=True,
        env={**os.environ},
    )
    try:
        assert parent.stdout is not None and parent.stdout.readline().strip() == "ready"
        wait_until(program_running, 2.0, "programa en marcha")
        assert parent.stdout.readline().strip() == "exiting"
        exiting_ns = now_ns()
        wait_until(lambda: not program_running(), stop_s + 5.0, "programa parado")
        stopped_after_s = (now_ns() - exiting_ns) / 1e9
        protective_stop = protective_stop_appears()
        print(
            f"parado {stopped_after_s * 1e3:.0f} ms después de que el padre empezara a salir; "
            f"parada de protección: {protective_stop}"
        )
        assert parent.wait(timeout=30) == 0
        assert not protective_stop, "el follower quedó en parada de protección: lo paró el watchdog"
        assert stopped_after_s < stop_s, "parada demasiado lenta para ser la ordenada"
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait(timeout=10)
        release_protective_stop()


# El sondeo de `program_running()` abre una conexión RTDE nueva en cada llamada.
WATCHDOG_POLL_MARGIN_S = 0.3


def test_killed_streaming_process_is_stopped_by_robot_watchdog() -> None:
    """Muerte brusca del proceso de streaming (el hijo): lo para el watchdog del robot.

    Distinto del test del padre matado: ahí el hijo sigue vivo, lo detecta y para ordenadamente
    (enable=0 y `stop` por el Dashboard), sin parada de protección. Aquí no queda nadie que lo
    haga: solo el watchdog RTDE del programa del TP (`rtde_set_watchdog` sobre
    input_int_register_0, a 2 / stop_s Hz porque el controlador para tras ~2 periodos sin dato,
    acción "stop"). Su valor está fijo en el programa, así que la prueba de que es él quien para
    es la parada de protección que deja (C207): los registros guardan su último valor, y sin
    watchdog el programa seguiría con enable=1. Y el padre tiene que enterarse en la siguiente
    llamada.
    """
    stop_s = CONFIG.follower.watchdog.stop_s
    processes: list[multiprocessing.process.BaseProcess] = []

    def launch(follower: FollowerConfig, start_tolerance_rad: float, shared: SharedState) -> Worker:
        worker = spawn_worker(follower, start_tolerance_rad, shared)
        assert isinstance(worker, multiprocessing.process.BaseProcess)
        processes.append(worker)
        return worker

    core = UrFollowerCore(CONFIG, launch=launch)
    try:
        core.connect()
        q0 = core.get_joints()
        core.send_joints(q0)
        wait_until(lambda: core.status().state == FollowerState.RUN, 2.0, "RUN")
        assert program_running()
        processes[0].kill()
        killed_ns = now_ns()
        processes[0].join(timeout=5)
        assert processes[0].exitcode not in (0, None)

        wait_until(lambda: not program_running(), stop_s + 5.0, "programa parado")
        stopped_after_s = (now_ns() - killed_ns) / 1e9
        print(f"parado {stopped_after_s * 1e3:.0f} ms después de matar el proceso de streaming")
        with pytest.raises(FollowerStoppedError, match="terminó sin publicar el motivo"):
            core.send_joints(q0)
        with pytest.raises(FollowerStoppedError, match="terminó sin publicar el motivo"):
            core.get_joints()
        assert stopped_after_s <= stop_s + WATCHDOG_POLL_MARGIN_S
        assert protective_stop_appears(), "sin parada de protección: no lo paró el watchdog"
        core.disconnect()  # con el proceso ya muerto no debe fallar
    finally:
        core.disconnect()
        release_protective_stop()
