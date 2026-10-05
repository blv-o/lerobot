"""Confirmación del operador antes de conectar un robot, compartida por follower y leader.

El código no mueve nunca el robot a la posición inicial (eso se hace desde el Teach Pendant):
solo comprueba que está allí, para que la primera consigna no provoque un salto.
"""

import math
from collections.abc import Callable, Sequence

from ur_core.config import JOINT_NAMES


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
