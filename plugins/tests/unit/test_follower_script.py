"""El programa del follower para el TP (plugins/tp/follower_control.script) tiene la estructura
esperada: interpola hacia cada consigna, limita la velocidad y se queda en la última.

Su comportamiento real (servoj, stopj) solo se puede comprobar en URSim; aquí se comprueba el texto.
"""

import math
import re
from pathlib import Path

import pytest

SCRIPT_PATH = Path(__file__).parents[2] / "tp" / "follower_control.script"


@pytest.fixture
def script() -> str:
    return SCRIPT_PATH.read_text(encoding="utf-8")


def constant(script: str, name: str) -> str:
    match = re.search(rf"^{name} = (.+)$", script, re.MULTILINE)
    assert match is not None, f"falta la constante {name}"
    return match[1].strip()


def test_script_is_plain_ascii() -> None:
    """PolyScope carga el fichero tal cual: sin tildes no hay dudas de codificación."""
    assert SCRIPT_PATH.read_bytes().isascii()


def test_no_rtde_watchdog(script: str) -> None:
    """Si el PC calla, el robot se queda en la última consigna; lo para el operador desde el TP."""
    assert "rtde_set_watchdog" not in script


def test_motion_constants_and_servoj_arguments(script: str) -> None:
    assert float(constant(script, "DT_S")) == 0.008  # 125 Hz
    assert float(constant(script, "TARGET_PERIOD_S")) == 0.033  # LeRobot a 30 fps
    assert float(constant(script, "MAX_SPEED_RAD_S")) == pytest.approx(math.radians(60))
    assert constant(script, "MAX_STEP_RAD") == "MAX_SPEED_RAD_S * DT_S"
    assert float(constant(script, "LOOKAHEAD_S")) == 0.1
    assert float(constant(script, "GAIN")) == 300
    calls = re.findall(r"servoj\((\w+), 0, 0, (\w+), (\w+), (\w+)\)", script)
    assert calls == [("q", "DT_S", "LOOKAHEAD_S", "GAIN")] * 2 + [("cmd", "DT_S", "LOOKAHEAD_S", "GAIN")]


def test_holds_the_pose_then_follows_each_new_target_with_limited_steps_and_stops(script: str) -> None:
    """Pose leída una vez → espera a que cambie el contador del PC (no a un valor: los registros
    guardan el último tras desconectar) → quieto con enable=0 → con enable=1, tramo nuevo desde el
    último comando a cada cambio del contador, avance recortado al final del tramo y paso recortado
    a la velocidad máxima → stopj al volver enable a 0."""
    assert script.count("get_actual_joint_positions()") == 1
    order = [
        "get_actual_joint_positions()",
        "v0 = read_input_integer_register(1)",
        "while read_input_integer_register(1) == v0:",
        "while read_input_integer_register(0) == 0:",
        "while read_input_integer_register(0) == 1:",
        "if c != seen:",
        "start = [cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]]",
        "read_input_float_register(5)",
        "alpha = 0.0",
        "alpha = alpha + DT_S / TARGET_PERIOD_S",
        "if alpha > 1.0:",
        "alpha = 1.0",
        "if step > MAX_STEP_RAD:",
        "step = MAX_STEP_RAD",
        "step = -MAX_STEP_RAD",
        "servoj(cmd,",
        "stopj(2.0)",
    ]
    position = 0
    for fragment in order:
        position = script.find(fragment, position)
        assert position >= 0, f"falta o está fuera de orden: {fragment}"


def test_no_endless_loop_so_the_program_ends_when_disabled(script: str) -> None:
    assert "while True" not in script


def test_heartbeat_written_in_the_three_loops(script: str) -> None:
    """`connect()` del PC espera a ver latir el programa, esté en el bucle que esté."""
    loops = script.split("\nwhile ")[1:]
    assert len(loops) == 3
    assert all("write_output_integer_register(0, hb)" in loop.split("\nend")[0] for loop in loops)
