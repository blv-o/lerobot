"""Cliente de Dashboard (29999) contra un servidor TCP local."""

import socket
import threading
from collections.abc import Callable, Iterator

import pytest
from ur_core.dashboard import DashboardClient


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


class FakeSocket:
    """Socket que entrega `incoming` (b"" = el servidor cerró) y registra si se cerró."""

    def __init__(self, incoming: list[bytes]) -> None:
        self._incoming = incoming
        self.sent = b""
        self.closed = False

    def recv(self, _size: int) -> bytes:
        return self._incoming.pop(0) if self._incoming else b""

    def sendall(self, data: bytes) -> None:
        self.sent += data

    def close(self) -> None:
        self.closed = True


def test_dashboard_failed_welcome_closes_the_socket_and_next_connect_reconnects(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Si el socket quedara abierto y marcado como conectado, el siguiente `connect()` volvería
    sin hacer nada y el `stop` de respaldo iría a una conexión muerta."""
    dead = FakeSocket([b""])
    alive = FakeSocket([b"Connected\n", b"Stopped\n"])
    opened = [dead, alive]
    monkeypatch.setattr(socket, "create_connection", lambda *_args, **_kwargs: opened.pop(0))
    dashboard = DashboardClient("127.0.0.1")
    with pytest.raises(ConnectionError):
        dashboard.connect()
    assert dead.closed
    dashboard.connect()
    assert dashboard.send("stop") == "Stopped"
    assert alive.sent == b"stop\n"
