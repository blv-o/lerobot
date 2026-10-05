"""Clientes de Dashboard (29999) y de la interfaz secundaria (30002) contra un servidor TCP local."""

import socket
import threading
from collections.abc import Callable, Iterator

import pytest
from ur_core.dashboard import DashboardClient
from ur_core.secondary import send_script


@pytest.fixture
def tcp_server() -> Iterator[Callable[[Callable[[socket.socket], None]], int]]:
    """Arranca un servidor de una sola conexión en 127.0.0.1 y devuelve su puerto."""
    threads: list[threading.Thread] = []
    sockets: list[socket.socket] = []

    def start(handler: Callable[[socket.socket], None]) -> int:
        server = socket.create_server(("127.0.0.1", 0))
        sockets.append(server)

        def serve() -> None:
            conn, _ = server.accept()
            with conn:
                handler(conn)

        thread = threading.Thread(target=serve, daemon=True)
        thread.start()
        threads.append(thread)
        return server.getsockname()[1]

    yield start
    for thread in threads:
        thread.join(timeout=5)
    for server in sockets:
        server.close()


def _read_line(conn: socket.socket) -> bytes:
    data = b""
    while not data.endswith(b"\n"):
        chunk = conn.recv(1)
        if not chunk:
            break
        data += chunk
    return data


def test_dashboard_skips_welcome_and_returns_one_line_per_command(tcp_server) -> None:
    received: list[bytes] = []

    def handler(conn: socket.socket) -> None:
        conn.sendall(b"Connected: Universal Robots Dashboard Server\n")
        received.append(_read_line(conn))
        conn.sendall(b"true\n")
        received.append(_read_line(conn))
        conn.sendall(b"Stopped\n")

    port = tcp_server(handler)
    dashboard = DashboardClient("127.0.0.1", port=port)
    dashboard.connect()
    assert dashboard.send("is in remote control") == "true"
    assert dashboard.send("stop") == "Stopped"
    dashboard.close()
    assert received == [b"is in remote control\n", b"stop\n"]


def test_dashboard_connect_and_close_are_idempotent(tcp_server) -> None:
    port = tcp_server(lambda conn: conn.sendall(b"Connected\n"))
    dashboard = DashboardClient("127.0.0.1", port=port)
    dashboard.connect()
    dashboard.connect()
    dashboard.close()
    dashboard.close()


def test_dashboard_raises_if_server_closes(tcp_server) -> None:
    def handler(conn: socket.socket) -> None:
        conn.sendall(b"Connected\n")
        _read_line(conn)  # cierra sin responder

    port = tcp_server(handler)
    dashboard = DashboardClient("127.0.0.1", port=port)
    dashboard.connect()
    with pytest.raises(ConnectionError):
        dashboard.send("stop")
    dashboard.close()


def test_send_script_delivers_whole_program_with_final_newline(tcp_server) -> None:
    received: list[bytes] = []
    done = threading.Event()

    def handler(conn: socket.socket) -> None:
        data = b""
        while chunk := conn.recv(4096):
            data += chunk
        received.append(data)
        done.set()

    port = tcp_server(handler)
    send_script("127.0.0.1", 'def prog():\n  textmsg("á")\nend', port=port)
    assert done.wait(timeout=5)
    assert received == ['def prog():\n  textmsg("á")\nend\n'.encode()]
