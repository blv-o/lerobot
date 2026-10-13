"""Confirmación del operador antes de conectar un robot, compartida por follower y leader.

El código no mueve nunca el robot a la posición inicial (eso se hace desde el Teach Pendant):
solo comprueba que está allí, para que la primera consigna no provoque un salto.
"""

import logging
import math
from collections.abc import Callable, Sequence
from typing import Any, Protocol

import rtde.rtde as rtde
from rtde.rtde import RTDEException

from ur_core.clock import now_ns
from ur_core.config import JOINT_NAMES

log = logging.getLogger("ur_core.startup")

RTDE_PORT = 30004


class RtdeConnection(Protocol):
    """Lo que follower y leader usan del cliente RTDE oficial (`rtde.rtde.RTDE`)."""

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def send_output_setup(self, variables: list[str], types: list[str], frequency: float) -> bool: ...
    def send_input_setup(self, variables: list[str], types: list[str]) -> Any: ...
    def send_start(self) -> bool: ...
    def send(self, input_data: Any) -> Any: ...
    def receive(self) -> Any: ...
    def receive_buffered(self) -> Any: ...


def rtde_connection(ip: str) -> RtdeConnection:
    return rtde.RTDE(ip, RTDE_PORT)


def confirm_and_check(
    prompt: str, checks: Callable[[], list[str]], ask: Callable[[str], str] = input
) -> bool:
    """Pregunta `yes/no` hasta que todo esté bien (True) o el operador diga `no` (False).

    `checks()` devuelve los problemas encontrados ([] = todo OK); se muestran y se vuelve a
    preguntar, para que el operador lo corrija en el TP sin relanzar el comando.
    """
    while True:
        answer = ask(prompt).strip().lower()
        if answer == "no":
            return False
        if answer != "yes":
            continue
        failures = checks()
        if not failures:
            return True
        for failure in failures:
            print(failure)


def confirm_start(prompt: str, checks: Callable[[], list[str]], ask: Callable[[str], str] = input) -> None:
    """`confirm_and_check` para los adaptadores: si el operador dice `no`, LeRobot termina sin conectar.

    `SystemExit` y no `KeyboardInterrupt`: `lerobot-teleoperate` conecta fuera de su `try` y
    `lerobot-record` no la captura, así que salía una traza larga por una respuesta normal.
    """
    if not confirm_and_check(prompt, checks, ask=ask):
        log.warning("arranque cancelado por el operador t_ns=%d", now_ns())
        raise SystemExit("arranque cancelado por el operador")


def start_pose_text(start_pose_rad: Sequence[float], tolerance_rad: float) -> str:
    """Posición inicial en grados, como la escribe el operador en el YAML."""
    pose = ", ".join(f"{round(math.degrees(q), 1):g}" for q in start_pose_rad)
    return f"[{pose}]° ± {round(math.degrees(tolerance_rad), 1):g}°"


def pose_errors(
    name: str, q_rad: Sequence[float], start_pose_rad: Sequence[float], tolerance_rad: float
) -> list[str]:
    """Una línea por articulación fuera de `start_pose ± tolerance`, en grados para el operador."""
    errors = []
    for joint, q, start in zip(JOINT_NAMES, q_rad, start_pose_rad, strict=True):
        error_rad = abs(q - start)
        if error_rad > tolerance_rad:
            degrees = f"{math.degrees(error_rad):.1f}".replace(".", ",")
            errors.append(f"{name} {joint}: {degrees}° fuera de tolerancia")
    return errors


def check_pose(
    name: str,
    ip: str,
    con: RtdeConnection,
    frequency_hz: float,
    start_pose_rad: Sequence[float],
    tolerance_rad: float,
) -> list[str]:
    """Problemas de posición inicial de un robot leído por RTDE. No lanza: cada fallo es una línea."""
    try:
        q_rad = read_pose(con, frequency_hz)
    except (OSError, RTDEException) as exc:
        return [f"{name}: no se puede leer la posición por RTDE en {ip}: {exc}"]
    return pose_errors(name, q_rad, start_pose_rad, tolerance_rad)


def read_pose(con: RtdeConnection, frequency_hz: float) -> list[float]:
    """Una sola `actual_q` por una conexión breve, que siempre se cierra."""
    try:
        start_output(con, ["actual_q"], ["VECTOR6D"], frequency_hz)
        return list(receive_packet(con).actual_q)
    finally:
        con.disconnect()


def start_output(
    con: RtdeConnection,
    variables: list[str],
    types: list[str],
    frequency_hz: float,
    inputs: tuple[list[str], list[str]] | None = None,
) -> Any:
    """Conecta, declara las recetas y arranca; los fallos del cliente oficial salen como RTDEException.

    Devuelve el objeto de la receta de entrada (`inputs`: nombres y tipos), o None sin ella.
    El cliente oficial abre el socket antes de negociar el protocolo y, si la negociación falla,
    lo deja abierto: quien llama debe cerrar la conexión (su `disconnect()` es seguro sin socket).
    """
    try:
        con.connect()
    except RTDEException as exc:
        # El puerto acepta pero el controlador no negocia (Docker publica el puerto aunque URSim
        # aún esté arrancando).
        raise RTDEException(f"{exc} (¿el controlador todavía está arrancando?)") from exc
    try:
        output_ok = con.send_output_setup(variables, types, frequency=frequency_hz)
        command = None if inputs is None else _input_setup(con, *inputs)
    except AttributeError as exc:
        # El cliente oficial hace `result.types` sobre la respuesta sin comprobar que llegó.
        raise RTDEException("el robot no respondió a la configuración RTDE") from exc
    if not output_ok:
        raise RTDEException(f"el robot rechazó la receta de salida {', '.join(variables)}")
    if not con.send_start():
        raise RTDEException("no se pudo iniciar la sincronización")
    return command


def _input_setup(con: RtdeConnection, variables: list[str], types: list[str]) -> Any:
    busy = (
        f"registros de entrada {', '.join(variables)} no disponibles "
        "(¿los usa otro cliente RTDE o un bus de campo?)"
    )
    try:
        command = con.send_input_setup(variables, types)
    except ValueError as exc:
        # El cliente oficial lanza ValueError al leer `IN_USE` en la respuesta, en vez de
        # devolver None.
        raise RTDEException(f"{busy}: {exc}") from exc
    if command is None:  # tipos distintos a los pedidos
        raise RTDEException(busy)
    return command


def receive_packet(con: RtdeConnection) -> Any:
    packet = con.receive()
    if packet is None:  # el cliente oficial devuelve None tras su DEFAULT_TIMEOUT sin datos
        raise RTDEException("sin datos del robot")
    return packet
