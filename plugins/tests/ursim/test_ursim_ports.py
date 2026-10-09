"""Los dos URSim de plugins/ursim/docker-compose.yml son alcanzables desde el host.

Cada URSim se publica en su propia IP de loopback, así
ambos usan los puertos estándar de UR sin campos de puerto en el YAML.
"""

import socket

import pytest

FOLLOWER_IP = "127.0.0.2"
LEADER_IP = "127.0.0.3"
RTDE_PORT = 30004
SECONDARY_PORT = 30002
DASHBOARD_PORT = 29999


@pytest.mark.ursim
@pytest.mark.parametrize(
    ("ip", "port"),
    [
        (FOLLOWER_IP, RTDE_PORT),
        (LEADER_IP, SECONDARY_PORT),  # el helper de test que mueve el URSim leader le sube un URScript
        (FOLLOWER_IP, DASHBOARD_PORT),
        (LEADER_IP, RTDE_PORT),
    ],
)
def test_port_accepts_connections(ip: str, port: int) -> None:
    with socket.create_connection((ip, port), timeout=5.0):
        pass


@pytest.mark.ursim
@pytest.mark.parametrize("ip", [FOLLOWER_IP, LEADER_IP])
def test_rtde_reports_controller_version(ip: str) -> None:
    """Comprueba que detrás del puerto hay un controlador UR de verdad, no solo un socket."""
    import rtde.rtde as rtde

    con = rtde.RTDE(ip, RTDE_PORT)
    con.connect()
    try:
        major, _minor, _bugfix, _build = con.get_controller_version()
        assert major == 5
    finally:
        con.disconnect()
