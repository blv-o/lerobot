"""Subida de un programa URScript por la interfaz secundaria de UR (puerto 30002).

El controlador compila y ejecuta el programa en cuanto lo recibe, y sigue corriendo aunque se
cierre el socket. Así el follower arranca su bucle de `servoj` sin un programa `.urp` hecho a
mano en PolyScope.
"""

import socket

SECONDARY_PORT = 30002


def send_script(host: str, script_text: str, port: int = SECONDARY_PORT, timeout_s: float = 5.0) -> None:
    # El controlador necesita el salto de línea final para dar el programa por completo.
    if not script_text.endswith("\n"):
        script_text += "\n"
    with socket.create_connection((host, port), timeout=timeout_s) as sock:
        sock.sendall(script_text.encode("utf-8"))
