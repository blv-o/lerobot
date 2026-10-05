"""Cliente mínimo del Dashboard Server de UR (puerto 29999).

Solo se usa para consultar (`is in remote control`) y para el `stop` de respaldo al parar el
follower. Encender, soltar frenos o desbloquear paradas se hace desde el Teach Pendant, nunca
desde aquí. Protocolo: una línea de texto por comando y una por respuesta, y una línea de
bienvenida al conectar.
"""

import socket

DASHBOARD_PORT = 29999


class DashboardClient:
    def __init__(self, host: str, port: int = DASHBOARD_PORT, timeout_s: float = 5.0) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self._sock: socket.socket | None = None
        self._buf = b""

    def connect(self) -> None:
        if self._sock is not None:
            return
        self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        self._buf = b""
        self._read_line()  # bienvenida

    def send(self, command: str) -> str:
        if self._sock is None:
            raise ConnectionError(f"Dashboard de {self.host} no conectado")
        self._sock.sendall(f"{command}\n".encode())
        return self._read_line()

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def _read_line(self) -> str:
        assert self._sock is not None
        while b"\n" not in self._buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError(f"el Dashboard de {self.host} cerró la conexión")
            self._buf += chunk
        line, _, self._buf = self._buf.partition(b"\n")
        return line.decode("utf-8", errors="replace").strip()
