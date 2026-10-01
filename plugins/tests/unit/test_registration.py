"""T0.3 / RF-21: los tipos `ur_follower` y `ur_leader` quedan registrados en LeRobot."""

import os
import shutil
import subprocess
import sys
from pathlib import Path

from lerobot.robots import RobotConfig
from lerobot.teleoperators import TeleoperatorConfig
from lerobot.utils.import_utils import register_third_party_plugins


def _env_scripts_path() -> str:
    """El ejecutable vive junto al Python del entorno (Scripts/ en Windows, bin/ en Linux),
    esté o no activado el entorno conda en la shell que lanza pytest."""
    env_dir = Path(sys.executable).parent
    return os.pathsep.join([str(env_dir), str(env_dir / "Scripts")])


def test_teleoperate_help_lists_both_types() -> None:
    exe = shutil.which("lerobot-teleoperate", path=_env_scripts_path())
    assert exe is not None, "lerobot-teleoperate no está instalado en este entorno"
    result = subprocess.run([exe, "--help"], capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stderr
    assert "ur_follower" in result.stdout
    assert "ur_leader" in result.stdout


def test_config_types_resolve_to_plugin_classes() -> None:
    register_third_party_plugins()
    assert RobotConfig.get_choice_class("ur_follower").__name__ == "UrFollowerConfig"
    assert TeleoperatorConfig.get_choice_class("ur_leader").__name__ == "UrLeaderConfig"
