"""Jitter del bucle del follower contra el URSim de plugins/ursim, en este PC.

    pytest plugins/tests -m timing -s

Necesita el URSim follower preparado como en los tests `ursim`: el programa `follower_tp.urp` en
su carpeta de programas y, solo para que el test le dé Play por el Dashboard antes de cada
escenario, el modo Remote. 60 s por escenario, a 125 Hz (el `t` de servoj del programa del TP),
sin carga y con carga de vídeo. La carga es la de `lerobot-record` con codificación en
directo (`--dataset.streaming_encoding=true --dataset.encoder_threads=2`): un hilo en el mismo
proceso que usa el follower, codificando una cámara 640×480 a 30 fps con PyAV (ffmpeg) y las
opciones de códec de LeRobot. El bucle de los joints corre en su propio proceso.

El periodo es el tiempo entre paquetes RTDE vistos por el bucle. `status()` da p50/p99 de una
ventana deslizante de 10 s y el máximo de toda la sesión; aquí se exige el peor p99 de todas
las ventanas, que es más estricto que el p99 de los 60 s completos.

Ojo al leer el resultado con URSim en Docker Desktop en el mismo PC: con la CPU saturada, los
paquetes RTDE se retienen a veces 100–400 ms antes de llegar a ningún proceso del PC (un lector
RTDE independiente con prioridad alta ve los mismos huecos y el `timestamp` del controlador no),
es decir, en el reenvío de Docker/WSL. Ese tramo no existe con un UR real por Ethernet.
"""

import threading
import time
from collections.abc import Iterator

import pytest
from conftest import CONFIG, VIDEO_FPS, encode_video
from ur_core import FollowerState, UrFollowerCore
from ur_core.clock import now_ns

DURATION_S = 60.0
SEND_HZ = 30
P99_LIMIT_S = 0.010
MAX_LIMIT_S = 0.016

# El programa del follower en Play antes de cada escenario (tras una parada termina).
pytestmark = [pytest.mark.timing, pytest.mark.usefixtures("follower_program_playing")]


@pytest.fixture
def video_load(request: pytest.FixtureRequest) -> Iterator[None]:
    if not request.param:
        yield
        return
    stop = threading.Event()
    frames_encoded = [0]
    worker = threading.Thread(target=encode_video, args=(stop, frames_encoded), daemon=True)
    worker.start()
    # svtav1 tarda unos segundos en inicializarse: medir solo con el códec ya codificando.
    deadline_ns = now_ns() + 30_000_000_000
    while frames_encoded[0] < VIDEO_FPS:
        assert now_ns() < deadline_ns, "la carga de vídeo no arrancó"
        time.sleep(0.1)
    t0_ns, frames0 = now_ns(), frames_encoded[0]
    yield
    fps = (frames_encoded[0] - frames0) / ((now_ns() - t0_ns) / 1e9)
    stop.set()
    worker.join(timeout=10)
    print(f"vídeo codificado a {fps:.1f} fps")


@pytest.mark.parametrize("video_load", [False, True], ids=["sin_video", "con_video"], indirect=True)
def test_loop_period_jitter(video_load: None) -> None:
    core = UrFollowerCore(CONFIG)
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
    print(
        f"\np50 {status.period_p50_s * 1e3:.2f} ms, peor p99 {worst_p99_s * 1e3:.2f} ms "
        f"(límite {P99_LIMIT_S * 1e3:.1f}), máx {status.period_max_s * 1e3:.2f} ms (límite {MAX_LIMIT_S * 1e3:.1f})"
    )
    assert status.state != FollowerState.STOP, status.stop_reason
    assert worst_p99_s < P99_LIMIT_S
    assert status.period_max_s < MAX_LIMIT_S
