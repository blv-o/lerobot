"""El URScript del follower sale de la configuración y tiene la estructura esperada.

Su comportamiento real (servoj, watchdog, stopj) solo se puede comprobar en URSim; aquí se
comprueba que la plantilla queda completa y en el orden correcto.
"""

import re
import tomllib
from pathlib import Path

import pytest
import ur_core
from ur_core.config import ServoConfig, WatchdogConfig
from ur_core.follower_script import render_follower_script

SERVO = ServoConfig(hz=125, gain=300, lookahead_s=0.1, max_joint_speed_rad_s=1.0, target_period_s=0.033)
WATCHDOG = WatchdogConfig(hold_s=0.1, stop_s=0.5)


@pytest.fixture
def script() -> str:
    return render_follower_script(SERVO, WATCHDOG)


def test_all_template_tokens_are_replaced(script: str) -> None:
    assert re.search(r"__\w+__", script) is None


def test_watchdog_on_enable_register_at_two_over_stop_time(script: str) -> None:
    """El controlador para tras ~2 periodos sin dato (medido en URSim): 2 / stop_s para en stop_s."""
    assert 'rtde_set_watchdog("input_int_register_0", 4.0, "stop")' in script


def test_servoj_uses_servo_period_lookahead_and_gain(script: str) -> None:
    calls = re.findall(r"servoj\((\w+), 0, 0, ([^)]*)\)", script)
    assert [target for target, _ in calls] == ["q", "tq"]
    assert all(args == "0.008, 0.1, 300" for _, args in calls)


def test_wait_holds_pose_read_once_then_follows_registers_then_stops(script: str) -> None:
    """Orden: watchdog → leer pose una vez → mantenerla con enable=0 → seguir registros con
    enable=1 → stopj al volver a 0. Leer la pose en cada ciclo la dejaría derivar."""
    assert script.count("get_actual_joint_positions()") == 1
    order = [
        "rtde_set_watchdog",
        "get_actual_joint_positions()",
        "while read_input_integer_register(0) == 0:",
        "servoj(q,",
        "while read_input_integer_register(0) == 1:",
        "read_input_float_register(5)",
        "servoj(tq,",
        "stopj(",
    ]
    positions = [script.index(fragment) for fragment in order]
    assert positions == sorted(positions)


def test_no_endless_loop_so_the_program_ends_when_disabled(script: str) -> None:
    assert "while True" not in script


def test_heartbeat_written_in_both_loops(script: str) -> None:
    assert script.count("write_output_integer_register(0, hb)") >= 3


def test_script_is_declared_as_package_data() -> None:
    """Sin esto, una instalación no editable de ur_core se quedaría sin el URScript."""
    pyproject = Path(ur_core.__file__).parents[1] / "pyproject.toml"
    data = tomllib.loads(pyproject.read_text(encoding="utf-8"))
    assert "*.script" in data["tool"]["setuptools"]["package-data"]["ur_core"]
