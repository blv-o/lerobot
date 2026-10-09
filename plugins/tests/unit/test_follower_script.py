"""El programa del follower para el TP (plugins/tp/follower_control.script) tiene la estructura
esperada y los mismos valores que la plantilla del YAML.

Su comportamiento real (servoj, watchdog, stopj) solo se puede comprobar en URSim. Sus valores
están escritos a mano, así que aquí se comprueba que no se separan de la plantilla.
"""

import re
from pathlib import Path

import pytest
from ur_core import load_config

PLUGINS = Path(__file__).parents[2]
SCRIPT_PATH = PLUGINS / "tp" / "follower_control.script"
CONFIG = load_config(PLUGINS / "configs" / "ur_config.yaml")


@pytest.fixture
def script() -> str:
    return SCRIPT_PATH.read_text(encoding="utf-8")


def test_script_is_plain_ascii() -> None:
    """PolyScope carga el fichero tal cual: sin tildes no hay dudas de codificación."""
    assert SCRIPT_PATH.read_bytes().isascii()


def test_watchdog_on_enable_register_at_two_over_stop_time(script: str) -> None:
    """El controlador para tras ~2 periodos sin dato (medido en URSim): 2 / stop_s para en stop_s."""
    match = re.search(r'rtde_set_watchdog\("input_int_register_0", ([\d.]+), "stop"\)', script)
    assert match is not None
    assert float(match[1]) == pytest.approx(2 / CONFIG.follower.watchdog.stop_s)


def test_servoj_period_matches_servo_hz(script: str) -> None:
    calls = re.findall(r"servoj\((\w+), 0, 0, ([\d.]+), ([\d.]+), ([\d.]+)\)", script)
    assert [target for target, *_ in calls] == ["q", "q", "tq"]
    assert all(float(t_s) == pytest.approx(1 / CONFIG.follower.servo.hz) for _, t_s, _, _ in calls)
    assert len({(lookahead_s, gain) for _, _, lookahead_s, gain in calls}) == 1


def test_watchdog_starts_after_the_pc_counter_changes_then_holds_follows_and_stops(script: str) -> None:
    """Orden: leer la pose una vez → esperar a que cambie el contador del PC (no a un valor: los
    registros guardan el último tras desconectar) → watchdog → mantener la pose con enable=0 →
    seguir los registros con enable=1 → stopj al volver a 0."""
    assert script.count("get_actual_joint_positions()") == 1
    order = [
        "get_actual_joint_positions()",
        "v0 = read_input_integer_register(1)",
        "while read_input_integer_register(1) == v0:",
        "rtde_set_watchdog",
        "while read_input_integer_register(0) == 0:",
        "while read_input_integer_register(0) == 1:",
        "read_input_float_register(5)",
        "servoj(tq,",
        "stopj(",
    ]
    positions = [script.index(fragment) for fragment in order]
    assert positions == sorted(positions)


def test_no_endless_loop_so_the_program_ends_when_disabled(script: str) -> None:
    assert "while True" not in script


def test_heartbeat_written_in_both_enable_loops(script: str) -> None:
    assert script.count("write_output_integer_register(0, hb)") == 2
