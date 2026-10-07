"""Follower contra el URSim de plugins/ursim (127.0.0.2), preparado a mano desde PolyScope.

Antes de lanzarlos (ver plugins/ursim/README.md): robot encendido con frenos sueltos, Remote
Control activado y en modo Remote, y en la posición inicial de plugins/configs/ur_config.yaml.

Con `UR_RECORD_TRACES=1` además se graban en plugins/tests/traces/ las trazas RTDE del
follower que usan los tests de replay (sin esa variable no se escribe nada).
"""

import dataclasses
import json
import math
import multiprocessing.process
import os
import subprocess
import sys
import threading
import time
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pytest
import rtde.rtde as rtde
from ur_core import FollowerState, FollowerStoppedError, UrFollowerCore, load_config
from ur_core.clock import now_ns
from ur_core.config import FollowerConfig
from ur_core.dashboard import DashboardClient
from ur_core.follower import Worker, spawn_worker
from ur_core.streaming import OUTPUT_NAMES, OUTPUT_TYPES, RTDE_PORT, RUNTIME_PLAYING, SharedState

PLUGINS = Path(__file__).parents[2]
CONFIG_PATH = PLUGINS / "configs" / "ur_config.yaml"
TRACES = PLUGINS / "tests" / "traces"
CONFIG = load_config(CONFIG_PATH)
FOLLOWER_IP = CONFIG.follower.ip
SINE_AMPLITUDE_RAD = 0.2
SINE_FREQUENCY_HZ = 0.2
SEND_HZ = 30
SINE_DURATION_S = 2 / SINE_FREQUENCY_HZ  # dos periodos completos
SHAPE_RMS_LIMIT_RAD = 0.02
DELAY_LIMIT_S = 0.2
SETTLE_S = 0.5
MAX_SEARCHED_DELAY_MS = 300  # por encima del límite, para que un retraso excesivo se vea como tal

pytestmark = pytest.mark.ursim


def dashboard(command: str) -> str:
    client = DashboardClient(FOLLOWER_IP)
    client.connect()
    try:
        return client.send(command)
    finally:
        client.close()


def runtime_state() -> int:
    """Estado del programa por RTDE. `programState` del Dashboard no sirve: solo informa del
    programa .urp cargado, no de un URScript subido por la interfaz secundaria."""
    con = rtde.RTDE(FOLLOWER_IP, RTDE_PORT)
    con.connect()
    try:
        assert con.send_output_setup(["runtime_state"], ["UINT32"], frequency=CONFIG.follower.servo.hz)
        assert con.send_start()
        return con.receive().runtime_state
    finally:
        con.disconnect()


def program_running() -> bool:
    return runtime_state() == RUNTIME_PLAYING


def wait_until(predicate: Any, timeout_s: float, what: str) -> None:
    deadline_ns = now_ns() + round(timeout_s * 1e9)
    while not predicate():
        assert now_ns() < deadline_ns, f"no se cumplió a tiempo: {what}"
        time.sleep(0.01)


# El controlador rechaza desbloquear una parada de protección antes de 5 s desde que ocurrió.
PROTECTIVE_STOP_UNLOCK_DELAY_S = 5.0
PROTECTIVE_STOP_RELEASE_TIMEOUT_S = 10.0
# Medido en URSim: el Dashboard informa de la parada 0,25-0,5 s después de que pare el programa.
PROTECTIVE_STOP_APPEAR_TIMEOUT_S = 2.0


def protective_stop_appears() -> bool:
    """Espera a que el Dashboard informe de la parada; False si no llega (el test falló antes)."""
    deadline_ns = now_ns() + round(PROTECTIVE_STOP_APPEAR_TIMEOUT_S * 1e9)
    while now_ns() < deadline_ns:
        if dashboard("safetystatus") == "Safetystatus: PROTECTIVE_STOP":
            return True
        time.sleep(0.1)
    return False


def release_protective_stop() -> None:
    """Devuelve el URSim a NORMAL si quedó en parada de protección.

    Cuando salta el watchdog RTDE, el controlador da C207 "Fieldbus input disconnected" y entra
    en parada de protección; sin desbloquearla, los tests siguientes no pueden arrancar el
    follower. En un robot real la desbloquea el operario desde el teach pendant.
    """
    if not protective_stop_appears():
        return
    # La parada ocurrió antes de verla: esperar desde ahora garantiza los 5 s.
    time.sleep(PROTECTIVE_STOP_UNLOCK_DELAY_S)
    reply = dashboard("unlock protective stop")
    assert reply.startswith("Protective stop releasing"), f"desbloqueo rechazado: {reply!r}"
    wait_until(
        lambda: dashboard("safetystatus") == "Safetystatus: NORMAL",
        PROTECTIVE_STOP_RELEASE_TIMEOUT_S,
        "Safetystatus: NORMAL",
    )


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
    """Remote Control real por el Dashboard y posición inicial: lo que pide el arranque."""
    assert dashboard("is in remote control") == "true"
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


def test_armed_follower_stays_still_in_wait_with_script_playing(core: UrFollowerCore) -> None:
    """Si runtime_state no fuera PLAYING (= 2) con el URScript corriendo, el bucle pararía."""
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


# El sondeo de `program_running()` abre una conexión RTDE nueva en cada llamada.
WATCHDOG_POLL_MARGIN_S = 0.3
# Watchdog largo para este test: a mitad de él el programa tiene que seguir en marcha, así se
# distingue del cierre del socket RTDE del hijo u otra vía que lo parase al instante.
LONG_WATCHDOG_STOP_S = 2.0


def test_killed_streaming_process_is_stopped_by_robot_watchdog() -> None:
    """Muerte brusca del proceso de streaming (el hijo): lo para el watchdog del robot.

    Distinto del test del padre matado: ahí el hijo sigue vivo, lo detecta y para ordenadamente
    (enable=0 y `stop` por el Dashboard). Aquí no queda nadie que lo haga: solo el watchdog RTDE
    del URScript (`rtde_set_watchdog` sobre input_int_register_0, frecuencia mínima
    2 / watchdog.stop_s porque el controlador para tras ~2 periodos sin dato, acción "stop").
    Con un `stop_s` largo se comprueba que es él quien para:
    sigue en marcha a mitad de `stop_s` y está parado poco después de `stop_s`. Y el padre tiene
    que enterarse en la siguiente llamada.
    """
    watchdog = dataclasses.replace(CONFIG.follower.watchdog, stop_s=LONG_WATCHDOG_STOP_S)
    assert watchdog.hold_s < watchdog.stop_s
    # El URScript toma la frecuencia del watchdog de esta config: tiene que ser la del follower.
    config = dataclasses.replace(CONFIG, follower=dataclasses.replace(CONFIG.follower, watchdog=watchdog))
    processes: list[multiprocessing.process.BaseProcess] = []

    def launch(follower: FollowerConfig, start_tolerance_rad: float, shared: SharedState) -> Worker:
        worker = spawn_worker(follower, start_tolerance_rad, shared)
        assert isinstance(worker, multiprocessing.process.BaseProcess)
        processes.append(worker)
        return worker

    core = UrFollowerCore(config, launch=launch)
    try:
        core.connect()
        q0 = core.get_joints()
        core.send_joints(q0)
        wait_until(lambda: core.status().state == FollowerState.RUN, 2.0, "RUN")

        def running_while_fed() -> bool:
            # Cada sondeo abre una conexión RTDE y puede tardar: sin consignas, el hijo pararía
            # por su cuenta (watchdog.stop_s) antes de matarlo.
            core.send_joints(q0)
            return program_running()

        wait_until(running_while_fed, 2.0, "programa en marcha")
        core.send_joints(q0)
        processes[0].kill()
        killed_ns = now_ns()
        processes[0].join(timeout=5)
        assert processes[0].exitcode not in (0, None)

        time.sleep(max(0.0, watchdog.stop_s / 2 - (now_ns() - killed_ns) / 1e9))
        running_at_half = program_running()
        probed_after_s = (now_ns() - killed_ns) / 1e9
        assert probed_after_s < watchdog.stop_s, (
            f"el sondeo a mitad del watchdog tardó {probed_after_s:.2f} s"
        )
        assert running_at_half, "el programa paró antes que el watchdog: lo paró otra vía"

        wait_until(lambda: not program_running(), watchdog.stop_s + 5.0, "programa parado")
        stopped_after_s = (now_ns() - killed_ns) / 1e9
        print(f"parado {stopped_after_s * 1e3:.0f} ms después de matar el proceso de streaming")
        with pytest.raises(FollowerStoppedError, match="terminó sin publicar el motivo"):
            core.send_joints(q0)
        with pytest.raises(FollowerStoppedError, match="terminó sin publicar el motivo"):
            core.get_joints()
        assert stopped_after_s <= watchdog.stop_s + WATCHDOG_POLL_MARGIN_S
        core.disconnect()  # con el proceso ya muerto no debe fallar
    finally:
        core.disconnect()
        release_protective_stop()
