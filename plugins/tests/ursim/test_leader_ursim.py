"""Leader contra el URSim de plugins/ursim (127.0.0.3): solo se lee, sin preparar nada en él."""

import time
from pathlib import Path

import pytest
from ur_core import UrLeaderCore, load_config

CONFIG = load_config(Path(__file__).parents[2] / "configs" / "ur_config.yaml")

pytestmark = pytest.mark.ursim


def test_leader_reads_joints_while_the_controller_timestamp_advances() -> None:
    leader = UrLeaderCore(CONFIG)
    leader.connect()
    try:
        time.sleep(3 * CONFIG.leader.timeout_s)  # sin timestamp nuevo, read_joints() lanzaría
        q_rad = leader.read_joints()
    finally:
        leader.disconnect()
    assert len(q_rad) == 6


def test_leader_check_start_reaches_the_robot() -> None:
    """La posición depende de cómo se dejó el URSim; aquí solo importa que la lectura funcione."""
    failures = UrLeaderCore(CONFIG).check_start()
    assert not any("no se puede leer" in failure for failure in failures), failures
