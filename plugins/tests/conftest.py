"""Lo que comparten los tests `ursim`: el URSim follower, sus trazas y la carga de vídeo.

`follower_program_playing` no es autouse (aquí se aplicaría también a los tests sin URSim): cada
archivo que lo necesita lo pide con `pytest.mark.usefixtures`. Las funciones se importan con
`from conftest import ...`.
"""

import json
import os
import socket
import threading
import time
from pathlib import Path
from typing import Any

import pytest
import rtde.rtde as rtde
from ur_core import load_config
from ur_core.clock import now_ns
from ur_core.follower import READ_HZ, RUNTIME_PLAYING
from ur_core.startup import RTDE_PORT

PLUGINS = Path(__file__).parents[1]
CONFIG_PATH = PLUGINS / "configs" / "ur_config.yaml"
CONFIG = load_config(CONFIG_PATH)
FOLLOWER_IP = CONFIG.follower.ip
DASHBOARD_PORT = 29999


def dashboard(command: str) -> str:
    """Un comando al Dashboard del URSim follower. Solo de test: el follower nunca lo usa."""
    with socket.create_connection((FOLLOWER_IP, DASHBOARD_PORT), timeout=5) as sock:
        stream = sock.makefile("rw", encoding="utf-8", newline="\n")
        stream.readline()  # bienvenida
        stream.write(command + "\n")
        stream.flush()
        return stream.readline().strip()


def runtime_state() -> int:
    """Estado del programa por RTDE: el mismo dato con el que el follower decide si sigue en marcha."""
    con = rtde.RTDE(FOLLOWER_IP, RTDE_PORT)
    con.connect()
    try:
        assert con.send_output_setup(["runtime_state"], ["UINT32"], frequency=READ_HZ)
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


def unlock_protective_stop() -> None:
    """Devuelve a NORMAL un URSim que un test anterior dejó en parada de protección. En un robot
    real la desbloquea el operario desde el TP."""
    # La parada ocurrió antes de verla: esperar desde ahora garantiza los 5 s.
    time.sleep(PROTECTIVE_STOP_UNLOCK_DELAY_S)
    reply = dashboard("unlock protective stop")
    assert reply.startswith("Protective stop releasing"), f"desbloqueo rechazado: {reply!r}"
    wait_until(
        lambda: dashboard("safetystatus") == "Safetystatus: NORMAL",
        PROTECTIVE_STOP_RELEASE_TIMEOUT_S,
        "Safetystatus: NORMAL",
    )


TRACES = PLUGINS / "tests" / "traces"


class TraceRecorder:
    """Graba el stream RTDE de un robot con una conexión de solo lectura propia.

    Con `UR_RECORD_TRACES=1`, `save()` escribe la traza en plugins/tests/traces/ para los tests
    de replay; sin esa variable no se escribe nada.
    """

    def __init__(self, ip: str, names: list[str], types: list[str], frequency_hz: float) -> None:
        self.packets: list[dict[str, Any]] = []
        self._names = names
        self._stop = threading.Event()
        self._con = rtde.RTDE(ip, RTDE_PORT)
        self._con.connect()
        assert self._con.send_output_setup(names, types, frequency=frequency_hz)
        assert self._con.send_start()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            pkt = self._con.receive()
            if pkt is not None:
                self.packets.append({name: _plain(getattr(pkt, name)) for name in self._names})

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=5)
        self._con.disconnect()

    def save(self, name: str) -> None:
        self.stop()
        if os.environ.get("UR_RECORD_TRACES") == "1":
            TRACES.mkdir(exist_ok=True)
            (TRACES / f"{name}.json").write_text(json.dumps(self.packets), encoding="utf-8")


def _plain(value: Any) -> Any:
    return list(value) if isinstance(value, list | tuple) else value


# Programa del URSim follower (carpeta plugins/ursim/programs/follower del checkout principal, no
# versionada): un nodo Script que carga plugins/tp/follower_control.script.
FOLLOWER_PROGRAM = "/ursim/programs/follower_tp.urp"
PROGRAM_START_TIMEOUT_S = 10.0


@pytest.fixture
def follower_program_playing() -> None:
    """Pone en marcha el programa del follower antes de cada test, como el operador con Play."""
    play_follower_program()


def play_follower_program() -> None:
    """Stop y Play del programa del follower por el Dashboard.

    Solo de test (el código del follower nunca lo hace) y con URSim en modo Remote. Siempre desde
    cero: una instancia que quedara de otro test podría estar ya habilitada.
    """
    if dashboard("safetystatus") == "Safetystatus: PROTECTIVE_STOP":
        unlock_protective_stop()
    if program_running():
        dashboard("stop")
        wait_until(lambda: not program_running(), PROGRAM_START_TIMEOUT_S, "programa anterior parado")
    replies = [dashboard(f"load {FOLLOWER_PROGRAM}")]

    def playing() -> bool:
        if program_running():
            return True
        replies.append(dashboard("play"))  # puede rechazarse mientras aún carga: se reintenta
        return False

    try:
        wait_until(playing, PROGRAM_START_TIMEOUT_S, "programa del follower en marcha")
    except AssertionError as exc:
        raise AssertionError(f"{exc}; respuestas del Dashboard: {replies}") from exc


# --- carga de vídeo -------------------------------------------------------------------------
# La de `lerobot-record` con codificación en directo (`--dataset.streaming_encoding=true
# --dataset.encoder_threads=2`): una cámara 640×480 a 30 fps codificada en un hilo del proceso.
VIDEO_FPS = 30
VIDEO_SIZE = (640, 480)
ENCODER_THREADS = 2  # el valor que recomienda lerobot-record para la codificación en directo


def encode_video(stop: threading.Event, frames_encoded: list[int]) -> None:
    """Codifica fotogramas de ruido (el peor caso para el códec) al ritmo de una cámara."""
    import av
    import numpy as np

    from lerobot.configs.video import VideoEncoderConfig

    encoder = VideoEncoderConfig()
    width, height = VIDEO_SIZE
    codec = av.CodecContext.create(encoder.vcodec, "w")
    codec.width, codec.height, codec.pix_fmt = width, height, encoder.pix_fmt
    codec.framerate = VIDEO_FPS
    codec.options = encoder.get_codec_options(ENCODER_THREADS, as_strings=True)
    rng = np.random.default_rng(0)
    frames = [rng.integers(0, 256, (height, width, 3), dtype=np.uint8) for _ in range(VIDEO_FPS)]
    period_ns = round(1e9 / VIDEO_FPS)
    next_ns = now_ns()
    while not stop.is_set():
        image = frames[frames_encoded[0] % len(frames)]
        codec.encode(av.VideoFrame.from_ndarray(image, format="rgb24").reformat(format=encoder.pix_fmt))
        frames_encoded[0] += 1
        next_ns += period_ns
        time.sleep(max(0.0, (next_ns - now_ns()) / 1e9))
