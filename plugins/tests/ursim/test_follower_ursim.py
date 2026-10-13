"""Follower contra el URSim de plugins/ursim (127.0.0.2), preparado a mano desde PolyScope.

Antes de lanzarlos (ver plugins/README.md): robot encendido con frenos sueltos, en modo
Remote, con el programa del follower en su carpeta de programas y en la posición inicial de
plugins/configs/ur_config.yaml. El modo Remote solo lo necesitan estos tests: antes de cada uno,
`follower_program_playing` pone el programa en marcha por el Dashboard, como haría el operador
con Play en el TP.

Con `UR_RECORD_TRACES=1` además se graban en plugins/tests/traces/ las trazas RTDE del
follower que usan los tests de replay (sin esa variable no se escribe nada).
"""

import math
import time
from collections.abc import Iterator, Sequence

import pytest
import rtde.rtde as rtde
from conftest import (
    CONFIG,
    FOLLOWER_IP,
    TraceRecorder,
    dashboard,
    play_follower_program,
    program_running,
    wait_until,
)
from ur_core import FollowerStoppedError, UrFollowerCore
from ur_core.clock import now_ns
from ur_core.follower import OUTPUT_NAMES, OUTPUT_TYPES, READ_HZ
from ur_core.startup import RTDE_PORT, read_pose, rtde_connection

SINE_AMPLITUDE_RAD = 0.2
SINE_FREQUENCY_HZ = 0.2
SEND_HZ = 30
SINE_DURATION_S = 2 / SINE_FREQUENCY_HZ  # dos periodos completos
SHAPE_RMS_LIMIT_RAD = 0.02
DELAY_LIMIT_S = 0.2
SETTLE_S = 0.5
MAX_SEARCHED_DELAY_MS = 300  # por encima del límite, para que un retraso excesivo se vea como tal
ENABLE_S = 0.3  # consignas antes de desconectar: el programa tiene que haber visto enable=1
STOP_DETECT_S = 1.0
STEP_RAD = 0.5
STEP_S = 1.5
MAX_SPEED_RAD_S = math.radians(60)  # la del programa del TP
SPEED_MARGIN = 1.05  # margen de la medida: posición y timestamp de paquetes consecutivos
STEP_REACHED_RAD = 0.01
SILENT_S = 3.0  # PC sin escribir con la conexión abierta
STILL_RAD = 1e-3
REARM_OFFSET_RAD = 0.1  # más que la tolerancia de la primera consigna (2·2°) y visible si se moviera
REARM_WAIT_S = 1.0

pytestmark = [pytest.mark.ursim, pytest.mark.usefixtures("follower_program_playing")]


def follower_recorder() -> TraceRecorder:
    return TraceRecorder(FOLLOWER_IP, OUTPUT_NAMES, OUTPUT_TYPES, READ_HZ)


@pytest.fixture
def core() -> Iterator[UrFollowerCore]:
    core = UrFollowerCore(CONFIG)
    yield core
    core.disconnect()


def send_for(core: UrFollowerCore, q_rad: Sequence[float], duration_s: float) -> None:
    """Reenvía la misma consigna a `SEND_HZ`, como LeRobot con el leader quieto."""
    for _ in range(round(duration_s * SEND_HZ)):
        core.send_joints(q_rad)
        time.sleep(1 / SEND_HZ)


def test_follower_is_ready_for_teleop() -> None:
    """Posición inicial leída por RTDE: lo que comprueba el arranque."""
    assert UrFollowerCore(CONFIG).check_start() == []


def test_rtde_mode_values_match_the_ones_the_follower_expects() -> None:
    """robot_mode RUNNING = 7 y safety_mode NORMAL = 1 con los tipos de la receta del follower."""
    con = rtde.RTDE(FOLLOWER_IP, RTDE_PORT)
    con.connect()
    try:
        assert con.send_output_setup(OUTPUT_NAMES, OUTPUT_TYPES, frequency=READ_HZ)
        assert con.send_start()
        pkt = con.receive()
    finally:
        con.disconnect()
    assert dashboard("robotmode") == "Robotmode: RUNNING" and pkt.robot_mode == 7
    assert dashboard("safetystatus") == "Safetystatus: NORMAL" and pkt.safety_mode == 1


def test_armed_follower_stays_still_with_program_playing(core: UrFollowerCore) -> None:
    """Si runtime_state no fuera PLAYING (= 2) con el programa del TP corriendo, get_joints lanzaría."""
    core.connect()
    q0 = core.get_joints()
    assert program_running()
    time.sleep(2.0)
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
    recorder = follower_recorder()
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
    send_for(core, q0, SETTLE_S)
    core.disconnect()
    # disconnect() para el programa; la traza tiene que acabar así: la reproduce test_replay.py.
    wait_until(lambda: not program_running(), 2.0, "programa parado")
    recorder.save("follower_sine")
    delays_s = [ms / 1000 for ms in range(0, MAX_SEARCHED_DELAY_MS + 1, 5)]
    delay_s = min(delays_s, key=lambda d: shape_rms_rad(samples, q0, d))
    rms_rad = shape_rms_rad(samples, q0, delay_s)
    print(
        f"forma RMS = {rms_rad:.4f} rad con retraso {delay_s * 1e3:.0f} ms "
        f"(sin descontarlo: {shape_rms_rad(samples, q0, 0.0):.4f} rad)"
    )
    assert dashboard("safetystatus") == "Safetystatus: NORMAL"
    assert rms_rad < SHAPE_RMS_LIMIT_RAD
    assert delay_s < DELAY_LIMIT_S


def test_large_step_is_followed_at_most_at_the_speed_limit(core: UrFollowerCore) -> None:
    """Escalón de 0,5 rad en wrist_3 reenviado a 30 Hz, y vuelta. La velocidad se mide entre
    paquetes RTDE consecutivos (Δactual_q / Δtimestamp del controlador), no entre llamadas."""
    recorder = follower_recorder()
    core.connect()
    q0 = core.get_joints()
    send_for(core, q0, ENABLE_S)  # la primera consigna tiene que estar cerca de la pose medida
    step = [*q0[:5], q0[5] + STEP_RAD]
    send_for(core, step, STEP_S)
    reached_rad = abs(core.get_joints()[5] - step[5])
    send_for(core, q0, STEP_S)
    back_rad = abs(core.get_joints()[5] - q0[5])
    core.disconnect()
    recorder.stop()
    pkts = recorder.packets
    speeds_rad_s = [
        abs(b["actual_q"][5] - a["actual_q"][5]) / (b["timestamp"] - a["timestamp"])
        for a, b in zip(pkts, pkts[1:], strict=False)
        if b["timestamp"] > a["timestamp"]
    ]
    print(f"velocidad máxima de wrist_3 = {math.degrees(max(speeds_rad_s)):.1f} °/s")
    assert reached_rad < STEP_REACHED_RAD and back_rad < STEP_REACHED_RAD
    assert max(speeds_rad_s) <= MAX_SPEED_RAD_S * SPEED_MARGIN
    assert dashboard("safetystatus") == "Safetystatus: NORMAL"


def test_silent_pc_with_rtde_open_leaves_the_robot_still_and_playing(core: UrFollowerCore) -> None:
    """Pausa de LeRobot o PC colgado: sin watchdog RTDE, el robot se queda en la última consigna."""
    core.connect()
    send_for(core, core.get_joints(), ENABLE_S)
    q0 = core.get_joints()
    t0_ns = now_ns()
    while now_ns() - t0_ns < SILENT_S * 1e9:
        # get_joints lanzaría FollowerStoppedError si el programa dejara de estar en Play.
        assert max(abs(a - b) for a, b in zip(core.get_joints(), q0, strict=True)) < STILL_RAD
        time.sleep(1 / SEND_HZ)
    assert program_running()


def test_rtde_cut_without_disconnect_leaves_the_program_playing(core: UrFollowerCore) -> None:
    """Conexión cerrada sin enable=0 (como un proceso matado): el programa sigue en Play; se para
    con Stop en el TP."""
    core.connect()
    send_for(core, core.get_joints(), ENABLE_S)
    core._connection().disconnect()  # cierra el socket del cliente RTDE sin escribir nada
    time.sleep(SILENT_S)
    assert program_running()
    assert dashboard("safetystatus") == "Safetystatus: NORMAL"


def test_program_stopped_from_pendant_stops_the_follower(core: UrFollowerCore) -> None:
    """El `stop` del Dashboard desde otra conexión hace lo mismo que el botón de parar del TP."""
    recorder = follower_recorder()
    core.connect()
    q0 = core.get_joints()
    send_for(core, q0, ENABLE_S)
    stop_ns = now_ns()
    dashboard("stop")
    # LeRobot sigue mandando consignas y se entera de la parada porque send_joints lanza.
    while True:
        try:
            core.send_joints(q0)
        except FollowerStoppedError as exc:
            reason = str(exc)
            break
        assert now_ns() - stop_ns < STOP_DETECT_S * 1e9
        time.sleep(1 / SEND_HZ)
    recorder.save("follower_stopped_from_pendant")
    assert "runtime_state" in reason, reason


def test_stop_and_play_while_lerobot_is_not_reading_never_moves_the_robot(core: UrFollowerCore) -> None:
    """Stop y Play en el TP durante una pausa de LeRobot (p. ej. `save_episode`): el PC lo ve al
    volver a leer, y el programa nuevo no se arma con el enable=1 que le llega aunque el PC escriba."""
    core.connect()
    q0 = core.get_joints()
    send_for(core, q0, ENABLE_S)
    play_follower_program()  # Stop y Play sin que el PC lea
    target = [*q0[:5], q0[5] + REARM_OFFSET_RAD]
    with pytest.raises(FollowerStoppedError, match="runtime_state"):
        core.send_joints(target)
    core._write(target, enable=1)  # lo que escribía el PC antes de revisar cada paquete
    time.sleep(REARM_WAIT_S)
    q_rad = read_pose(rtde_connection(FOLLOWER_IP), READ_HZ)
    assert max(abs(a - b) for a, b in zip(q_rad, q0, strict=True)) < STILL_RAD
    assert program_running()
