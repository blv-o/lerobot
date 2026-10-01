"""T0.2: los tres paquetes se instalan e importan.

LeRobot solo descubre un plugin si el nombre de su distribución empieza por
`lerobot_robot_`/`lerobot_teleoperator_` con guiones bajos y coincide con el paquete
importable (`register_third_party_plugins` en src/lerobot/utils/import_utils.py).
"""

import importlib
import importlib.metadata
from pathlib import Path

import pytest


@pytest.mark.parametrize(
    "dist_name",
    ["ur_teleop_core", "lerobot_robot_ur_follower", "lerobot_teleoperator_ur_leader"],
)
def test_distribution_is_installed_and_importable(dist_name: str) -> None:
    assert importlib.metadata.distribution(dist_name).metadata["Name"] == dist_name
    importlib.import_module(dist_name)


def test_core_does_not_import_lerobot() -> None:
    """Capa anticorrupción (principio 1): solo los plugins hablan con LeRobot."""
    import ur_teleop_core

    for path in Path(ur_teleop_core.__path__[0]).rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        assert "import lerobot" not in text and "from lerobot" not in text, path
