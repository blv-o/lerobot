"""Jitter del bucle del follower contra el URSim de plugins/ursim, en este PC.

    pytest plugins/tests -m timing -s

Necesita el follower preparado como en los tests `ursim`. 60 s por escenario: a `servo.hz` 125 y
500, sin carga y con carga de vídeo. La carga son dos procesos que codifican fotogramas 640×480 a
30 fps con libsvtav1 y los mismos ajustes que `lerobot-record` por defecto (preset 12, crf 30,
g 2): lo que hace el PC mientras graba con dos cámaras.

El periodo es el tiempo entre paquetes RTDE vistos por el bucle. `status()` da p50/p99 de una
ventana deslizante de 10 s y el máximo de toda la sesión; aquí se exige el peor p99 de todas
las ventanas, que es más estricto que el p99 de los 60 s completos.

Ojo al leer el resultado con URSim en Docker Desktop en el mismo PC: con la CPU saturada, los
paquetes RTDE se retienen a veces 100–400 ms antes de llegar a ningún proceso del PC (un lector
RTDE independiente con prioridad alta ve los mismos huecos y el `timestamp` del controlador no),
es decir, en el reenvío de Docker/WSL. Ese tramo no existe con un UR real por Ethernet.
"""

import dataclasses
import multiprocessing
import time
from collections.abc import Iterator
from multiprocessing.synchronize import Event
from pathlib import Path

import pytest
from ur_core import FollowerState, UrFollowerCore, load_config
from ur_core.clock import now_ns

CONFIG = load_config(Path(__file__).parents[2] / "configs" / "ur_config.yaml")
DURATION_S = 60.0
SEND_HZ = 30
VIDEO_PROCESSES = 2
VIDEO_FPS = 30
VIDEO_SIZE = (640, 480)
# servo.hz → (p99 máximo, máximo), en s.
LIMITS_S = {125: (0.010, 0.016), 500: (0.0025, 0.004)}

pytestmark = pytest.mark.timing


def _encode_video(stop: Event) -> None:
    """Codifica fotogramas de ruido (el peor caso para el codificador) al ritmo de una cámara."""
    import av
    import numpy as np

    width, height = VIDEO_SIZE
    codec = av.CodecContext.create("libsvtav1", "w")
    codec.width, codec.height, codec.pix_fmt = width, height, "yuv420p"
    codec.framerate = VIDEO_FPS
    codec.options = {"preset": "12", "crf": "30", "g": "2"}
    rng = np.random.default_rng(0)
    frames = [rng.integers(0, 256, (height, width, 3), dtype=np.uint8) for _ in range(VIDEO_FPS)]
    period_ns = round(1e9 / VIDEO_FPS)
    next_ns = now_ns()
    i = 0
    while not stop.is_set():
        frame = av.VideoFrame.from_ndarray(frames[i % len(frames)], format="rgb24").reformat(format="yuv420p")
        codec.encode(frame)  # los paquetes se descartan: solo interesa la carga
        i += 1
        next_ns += period_ns
        time.sleep(max(0.0, (next_ns - now_ns()) / 1e9))


@pytest.fixture
def video_load(request: pytest.FixtureRequest) -> Iterator[None]:
    if not request.param:
        yield
        return
    ctx = multiprocessing.get_context("spawn")
    stop = ctx.Event()
    workers = [ctx.Process(target=_encode_video, args=(stop,), daemon=True) for _ in range(VIDEO_PROCESSES)]
    for worker in workers:
        worker.start()
    time.sleep(2.0)  # que los codificadores estén en régimen antes de medir
    assert all(worker.is_alive() for worker in workers), "la carga de vídeo no arrancó"
    yield
    stop.set()
    for worker in workers:
        worker.join(timeout=10)


@pytest.mark.parametrize("video_load", [False, True], ids=["sin_video", "con_video"], indirect=True)
@pytest.mark.parametrize("servo_hz", [125, 500])
def test_loop_period_jitter(servo_hz: int, video_load: None) -> None:
    servo = dataclasses.replace(CONFIG.follower.servo, hz=servo_hz)
    config = dataclasses.replace(CONFIG, follower=dataclasses.replace(CONFIG.follower, servo=servo))
    core = UrFollowerCore(config)
    core.connect()
    try:
        q0 = core.get_joints()
        worst_p99_s = 0.0
        t0_ns = now_ns()
        next_stats_ns = t0_ns + 1_000_000_000
        while now_ns() - t0_ns < DURATION_S * 1e9:
            core.send_joints(q0)  # RUN durante toda la medida
            if now_ns() >= next_stats_ns:
                worst_p99_s = max(worst_p99_s, core.status().period_p99_s)
                next_stats_ns += 1_000_000_000
            time.sleep(1 / SEND_HZ)
        status = core.status()
    finally:
        core.disconnect()
    p99_limit_s, max_limit_s = LIMITS_S[servo_hz]
    print(
        f"\n{servo_hz} Hz: p50 {status.period_p50_s * 1e3:.2f} ms, peor p99 {worst_p99_s * 1e3:.2f} ms "
        f"(límite {p99_limit_s * 1e3:.1f}), máx {status.period_max_s * 1e3:.2f} ms (límite {max_limit_s * 1e3:.1f})"
    )
    assert status.state != FollowerState.STOP, status.stop_reason
    assert worst_p99_s < p99_limit_s
    assert status.period_max_s < max_limit_s
