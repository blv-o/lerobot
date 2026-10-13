"""Leader contra el URSim de plugins/ursim (127.0.0.3): solo se lee, sin preparar nada en él.

Con `UR_RECORD_TRACES=1` además se graba en plugins/tests/traces/ la traza RTDE del leader
quieto que usa el test de replay del timeout (sin esa variable no se escribe nada).
"""

import time

import pytest
from conftest import CONFIG, TraceRecorder
from ur_core import UrLeaderCore
from ur_core.leader import OUTPUT_NAMES, OUTPUT_TYPES
from ur_core.startup import READ_HZ

pytestmark = pytest.mark.ursim


def test_leader_reads_joints_while_the_controller_timestamp_advances() -> None:
    recorder = TraceRecorder(CONFIG.leader.ip, OUTPUT_NAMES, OUTPUT_TYPES, READ_HZ)
    leader = UrLeaderCore(CONFIG)
    leader.connect()
    try:
        time.sleep(3 * CONFIG.leader.timeout_s)  # sin timestamp nuevo, read_joints() lanzaría
        q_rad = leader.read_joints()
    finally:
        leader.disconnect()
        recorder.save("leader_still")
    assert len(q_rad) == 6


def test_leader_check_start_reaches_the_robot() -> None:
    """La posición depende de cómo se dejó el URSim; aquí solo importa que la lectura funcione."""
    failures = UrLeaderCore(CONFIG).check_start()
    assert not any("no se puede leer" in failure for failure in failures), failures
