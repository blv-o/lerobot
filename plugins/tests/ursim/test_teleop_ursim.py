"""Teleoperación URSim leader → URSim follower durante 60 s, como `lerobot-teleoperate`.

Mismo proceso que la CLI: adaptadores `UrLeader` y `UrFollower` con la confirmación respondida
por `ask`, y el bucle de `teleop_loop` (observación, acción del leader, envío al follower) a
`--fps` = 1000 / `target_period_ms`. Con la carga de vídeo de los tests `timing` en marcha, porque
el hilo de lectura del leader comparte el GIL con la codificación.

Solo para este test, el URSim leader se mueve con un URScript subido por 30002 (en un robot real
lo mueve el operador en freedrive). El código de los plugins nunca escribe al leader.
"""

import importlib.util
import math
import socket
import threading
import time
from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import pytest
import rtde.rtde as rtde
from lerobot_robot_ur_follower import UrFollower, UrFollowerConfig
from lerobot_teleoperator_ur_leader import UrLeader, UrLeaderConfig
from test_follower_ursim import (
    CONFIG,
    CONFIG_PATH,
    DELAY_LIMIT_S,
    MAX_SEARCHED_DELAY_MS,
    SHAPE_RMS_LIMIT_RAD,
    dashboard,
    follower_program_playing,  # noqa: F401  (autouse: programa del follower en Play)
    wait_until,
)
from ur_core import JOINT_NAMES, FollowerState
from ur_core.clock import now_ns
from ur_core.streaming import RTDE_PORT, RUNTIME_PLAYING

pytestmark = pytest.mark.ursim

LEADER_IP = CONFIG.leader.ip
SECONDARY_PORT = 30002
FPS = round(1 / CONFIG.follower.servo.target_period_s)  # --fps = 1000 / target_period_ms
TELEOP_S = 60.0
SETTLE_S = 3.0  # el follower sigue al leader de vuelta a la posición inicial
LEADER_AMPLITUDE_RAD = 0.1
LEADER_FREQUENCY_HZ = 0.2
LEADER_SERVO_S = 0.008
LEADER_MOTION_S = TELEOP_S + 20.0  # cubre el bucle; el script de vuelta a casa lo corta antes
LEADER_SCRIPT_TIMEOUT_S = 30.0
VIDEO_START_TIMEOUT_S = 30.0
VIDEO_MIN_FPS = 27.0
JOINT_KEYS = [f"{joint}.pos" for joint in JOINT_NAMES]


def _load_encode_video() -> Callable[[threading.Event, list[int]], None]:
    """La carga de vídeo de los tests `timing`, cargada por ruta (está en otra carpeta de tests)."""
    path = Path(__file__).parents[1] / "timing" / "test_follower_timing.py"
    spec = importlib.util.spec_from_file_location("follower_timing_video_load", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._encode_video


@contextmanager
def video_load() -> Iterator[list[int]]:
    """Codifica vídeo a 30 fps en un hilo de este proceso; da el contador de fotogramas."""
    stop = threading.Event()
    frames_encoded = [0]
    worker = threading.Thread(target=_load_encode_video(), args=(stop, frames_encoded), daemon=True)
    worker.start()
    try:
        # svtav1 tarda unos segundos en dar el primer fotograma: medir solo con el códec ya en marcha.
        wait_until(lambda: frames_encoded[0] >= 30, VIDEO_START_TIMEOUT_S, "la carga de vídeo arrancó")
        yield frames_encoded
    finally:
        stop.set()
        worker.join(timeout=10)


# --- URSim leader: solo de test ---------------------------------------------------------
def _joints(q_rad: Sequence[float]) -> str:
    return "[" + ", ".join(f"{q:.6f}" for q in q_rad) + "]"


def home_script() -> str:
    """Lleva el leader a la posición inicial; el `sleep` deja verlo en PLAYING aunque ya esté allí."""
    return f"def leader_home():\n  movej({_joints(CONFIG.start_pose_rad)}, a=1.0, v=0.5)\n  sleep(0.5)\nend\n"


def sine_script() -> str:
    """Seno suave en las 6 articulaciones alrededor de la posición inicial, empezando en ella."""
    targets = ", ".join(f"q0[{i}] + o" for i in range(len(JOINT_NAMES)))
    omega_rad_s = 2 * math.pi * LEADER_FREQUENCY_HZ
    return (
        "def leader_sine():\n"
        f"  q0 = {_joints(CONFIG.start_pose_rad)}\n"
        "  t = 0.0\n"
        f"  while t < {LEADER_MOTION_S}:\n"
        f"    o = {LEADER_AMPLITUDE_RAD} * sin({omega_rad_s:.6f} * t)\n"
        f"    servoj([{targets}], t={LEADER_SERVO_S}, lookahead_time=0.1, gain=300)\n"
        f"    t = t + {LEADER_SERVO_S}\n"
        "  end\n"
        "  stopj(2.0)\n"
        "end\n"
    )


def send_leader_script(script: str) -> None:
    """Un script nuevo por la interfaz secundaria sustituye al que esté corriendo."""
    with socket.create_connection((LEADER_IP, SECONDARY_PORT), timeout=5.0) as sock:
        sock.sendall(script.encode())


def run_leader_script(script: str) -> None:
    """Sube el script y espera a que empiece y termine (`runtime_state` por RTDE)."""
    con = rtde.RTDE(LEADER_IP, RTDE_PORT)
    con.connect()
    try:
        assert con.send_output_setup(["runtime_state"], ["UINT32"], frequency=CONFIG.leader.rtde_hz)
        assert con.send_start()
        send_leader_script(script)
        deadline_ns = now_ns() + round(LEADER_SCRIPT_TIMEOUT_S * 1e9)
        seen_playing = False
        while True:
            state = con.receive().runtime_state
            seen_playing = seen_playing or state == RUNTIME_PLAYING
            if seen_playing and state != RUNTIME_PLAYING:
                return
            assert now_ns() < deadline_ns, f"el script del leader no terminó (runtime_state={state})"
    finally:
        con.disconnect()


# --- teleoperación ---------------------------------------------------------------------
def answer_yes_once() -> Callable[[str], str]:
    """«yes» una vez; si el arranque falla y se vuelve a preguntar, falla en vez de colgarse."""
    asked = [False]

    def ask(prompt: str) -> str:
        assert not asked[0], "el arranque no pasó las comprobaciones (ver la salida)"
        asked[0] = True
        return "yes"

    return ask


Sample = tuple[float, list[float], list[float]]  # (t_s, pose medida del follower, acción del leader)


def teleop_for(leader: UrLeader, follower: UrFollower, duration_s: float) -> list[Sample]:
    """El bucle de `teleop_loop` de LeRobot, en el mismo orden: observar, leer el leader, enviar."""
    period_ns = round(1e9 / FPS)
    t0_ns = now_ns()
    next_ns = t0_ns
    samples: list[Sample] = []
    while (t_ns := now_ns()) - t0_ns < duration_s * 1e9:
        observation = follower.get_observation()
        action = leader.get_action()
        follower.send_action(action)
        samples.append(((t_ns - t0_ns) / 1e9, [observation[k] for k in JOINT_KEYS], [action[k] for k in JOINT_KEYS]))
        next_ns += period_ns
        time.sleep(max(0.0, (next_ns - now_ns()) / 1e9))
    return samples


def shape_rms_rad(samples: list[Sample], delay_s: float) -> float:
    """RMS entre la pose medida del follower y la acción del leader retrasada `delay_s`."""
    t_s = np.array([s[0] for s in samples])
    measured = np.array([s[1] for s in samples])
    actions = np.array([s[2] for s in samples])
    keep = t_s - delay_s >= t_s[0]
    reference = np.column_stack(
        [np.interp(t_s[keep] - delay_s, t_s, actions[:, j]) for j in range(actions.shape[1])]
    )
    return float(np.sqrt(np.mean((measured[keep] - reference) ** 2)))


def test_teleop_leader_to_follower_for_60_s_with_video_load(tmp_path: Path) -> None:
    """Sin `LeaderReadError` ni `FollowerStoppedError`, sin parada de protección y con buen seguimiento.

    Seguimiento con los mismos límites que el seno del follower (forma RMS < 0,02 rad con el
    retraso constante que mejor encaja; retraso < 200 ms): lo único que se añade es la lectura
    del leader (RTDE a 500 Hz, ~2 ms) y el muestreo del bucle a 30 Hz (≤ 1 periodo).
    """
    leader = UrLeader(UrLeaderConfig(config_path=CONFIG_PATH, calibration_dir=tmp_path / "leader"))
    follower = UrFollower(UrFollowerConfig(config_path=CONFIG_PATH, calibration_dir=tmp_path / "follower"))
    leader.ask = answer_yes_once()
    follower.ask = answer_yes_once()
    try:
        with video_load() as frames_encoded:
            run_leader_script(home_script())
            leader.connect()  # mismo orden que lerobot-teleoperate
            follower.connect()
            send_leader_script(sine_script())  # después de conectar: el arranque exige la posición inicial
            frames0, t0_ns = frames_encoded[0], now_ns()
            samples = teleop_for(leader, follower, TELEOP_S)
            video_fps = (frames_encoded[0] - frames0) / ((now_ns() - t0_ns) / 1e9)
            send_leader_script(home_script())
            teleop_for(leader, follower, SETTLE_S)
        status = follower.core.status()
    finally:
        leader.disconnect()
        follower.disconnect()
        run_leader_script(home_script())  # para el seno si el test falló a medias

    delays_s = [ms / 1000 for ms in range(0, MAX_SEARCHED_DELAY_MS + 1, 5)]
    delay_s = min(delays_s, key=lambda d: shape_rms_rad(samples, d))
    rms_rad = shape_rms_rad(samples, delay_s)
    actions = np.array([s[2] for s in samples])
    leader_ptp_rad = float(np.ptp(actions, axis=0).min())
    periods_ms = np.diff([s[0] for s in samples]) * 1e3
    print(
        f"\nsin LeaderReadError ni FollowerStoppedError en {TELEOP_S:.0f} s a {FPS} fps con vídeo a {video_fps:.1f} fps"
        f"\nforma RMS = {rms_rad:.4f} rad con retraso {delay_s * 1e3:.0f} ms "
        f"(sin descontarlo: {shape_rms_rad(samples, 0.0):.4f} rad); recorrido del leader {leader_ptp_rad:.3f} rad"
        f"\nperiodo del bucle p50/p99/máx = {np.percentile(periods_ms, 50):.2f}/"
        f"{np.percentile(periods_ms, 99):.2f}/{periods_ms.max():.2f} ms"
        f"\nperiodo del follower p50/p99/máx = {status.period_p50_s * 1e3:.2f}/"
        f"{status.period_p99_s * 1e3:.2f}/{status.period_max_s * 1e3:.2f} ms"
    )
    assert status.state != FollowerState.STOP, status.stop_reason
    assert dashboard("safetystatus") == "Safetystatus: NORMAL"
    assert video_fps >= VIDEO_MIN_FPS, "sin carga de vídeo real el test no prueba nada"
    assert leader_ptp_rad >= LEADER_AMPLITUDE_RAD, "el leader no se movió: el seguimiento no prueba nada"
    assert rms_rad < SHAPE_RMS_LIMIT_RAD
    assert delay_s < DELAY_LIMIT_S
