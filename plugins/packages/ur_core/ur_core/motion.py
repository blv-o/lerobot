"""Funciones puras que convierten consignas a ~30 Hz en comandos de servo a `servo.hz`.

LeRobot manda una consigna cada `target_period_s`, pero el `servoj` del UR necesita una cada
ciclo RTDE: sin repartir el salto entre ciclos el robot se movería a tirones.
"""

from collections.abc import Sequence


def interpolate(
    prev_target_rad: Sequence[float],
    new_target_rad: Sequence[float],
    elapsed_s: float,
    period_s: float,
) -> list[float]:
    """Punto de la recta prev → new tras `elapsed_s` de un recorrido de `period_s`.

    `elapsed_s` se recorta a [0, period_s]: si la consigna siguiente llega tarde, el robot se
    queda en la última en vez de extrapolar hacia una posición que nadie ha pedido.
    """
    if period_s <= 0:
        raise ValueError(f"period_s debe ser > 0 (vale {period_s})")
    alpha = min(max(elapsed_s / period_s, 0.0), 1.0)
    return [p + alpha * (n - p) for p, n in zip(prev_target_rad, new_target_rad, strict=True)]


def limit_step(cmd_rad: Sequence[float], last_cmd_rad: Sequence[float], max_step_rad: float) -> list[float]:
    """Recorta el incremento de cada articulación respecto al último comando a ±`max_step_rad`.

    Es el límite de velocidad articular: ante un escalón grande el robot avanza a velocidad
    máxima en lugar de saltar.
    """
    return [
        last + min(max(cmd - last, -max_step_rad), max_step_rad)
        for cmd, last in zip(cmd_rad, last_cmd_rad, strict=True)
    ]
