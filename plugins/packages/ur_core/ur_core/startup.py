"""Confirmación del operador antes de conectar un robot, compartida por follower y leader.

El código no mueve nunca el robot a la posición inicial (eso se hace desde el Teach Pendant):
solo comprueba que está allí, para que la primera consigna no provoque un salto.
"""

import math
from collections.abc import Callable, Sequence
from typing import Any

from rtde.rtde import RTDEException

from ur_core.config import JOINT_NAMES
from ur_core.streaming import RtdeConnection


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


def start_output(con: RtdeConnection, variables: list[str], types: list[str], frequency_hz: float) -> None:
    """Conecta y arranca una receta de salida; los fallos del cliente oficial salen como RTDEException.

    El cliente oficial abre el socket antes de negociar el protocolo y, si la negociación falla,
    lo deja abierto: quien llama debe cerrar la conexión (su `disconnect()` es seguro sin socket).
    """
    con.connect()
    try:
        output_ok = con.send_output_setup(variables, types, frequency=frequency_hz)
    except AttributeError as exc:
        # El cliente oficial hace `result.types` sobre la respuesta sin comprobar que llegó.
        raise RTDEException("el robot no respondió a la configuración RTDE") from exc
    if not output_ok:
        raise RTDEException(f"el robot rechazó la receta de salida {', '.join(variables)}")
    if not con.send_start():
        raise RTDEException("no se pudo iniciar la sincronización")


def receive_packet(con: RtdeConnection) -> Any:
    packet = con.receive()
    if packet is None:  # el cliente oficial devuelve None tras su DEFAULT_TIMEOUT sin datos
        raise RTDEException("sin datos del robot")
    return packet
