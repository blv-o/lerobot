"""Separación entre `ur_core` y los plugins de LeRobot."""

from pathlib import Path


def test_core_does_not_import_lerobot() -> None:
    """Capa anticorrupción: solo los plugins hablan con LeRobot."""
    import ur_core

    for path in Path(ur_core.__path__[0]).rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        assert "import lerobot" not in text and "from lerobot" not in text, path
