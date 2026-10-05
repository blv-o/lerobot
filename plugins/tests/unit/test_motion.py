"""Interpolación lineal entre consignas y límite de incremento por ciclo."""

import math

import pytest
from ur_core.motion import interpolate, limit_step

PREV = [0.0, -1.0, 1.0, 0.0, 0.5, -0.5]
NEW = [0.3, -0.7, 1.6, -0.3, 0.5, 0.1]
PERIOD_S = 0.033


def test_interpolate_starts_at_previous_and_ends_at_new() -> None:
    assert interpolate(PREV, NEW, 0.0, PERIOD_S) == pytest.approx(PREV)
    assert interpolate(PREV, NEW, PERIOD_S, PERIOD_S) == pytest.approx(NEW)


def test_interpolate_is_linear_in_between() -> None:
    mid = interpolate(PREV, NEW, PERIOD_S / 2, PERIOD_S)
    assert mid == pytest.approx([(p + n) / 2 for p, n in zip(PREV, NEW, strict=True)])


def test_interpolate_never_extrapolates() -> None:
    """Si la consigna siguiente llega tarde, la salida se queda en la consigna: no la sobrepasa."""
    assert interpolate(PREV, NEW, 10 * PERIOD_S, PERIOD_S) == pytest.approx(NEW)
    assert interpolate(PREV, NEW, -1.0, PERIOD_S) == pytest.approx(PREV)


def test_interpolate_is_continuous_over_a_stream_of_regular_targets() -> None:
    """Consignas regulares cada 33 ms muestreadas a 125 Hz: la salida no salta y llega a cada una."""
    servo_dt_s = 1 / 125
    targets = [[0.01 * k] * 6 for k in range(10)]
    out: list[list[float]] = []
    prev = targets[0]
    for k in range(1, len(targets)):
        t_s = 0.0
        while t_s < PERIOD_S:
            out.append(interpolate(prev, targets[k], t_s, PERIOD_S))
            t_s += servo_dt_s
        assert interpolate(prev, targets[k], PERIOD_S, PERIOD_S) == pytest.approx(targets[k])
        prev = targets[k]
    max_jump = max(abs(b[0] - a[0]) for a, b in zip(out, out[1:], strict=False))
    assert max_jump <= 0.01 * servo_dt_s / PERIOD_S + 1e-12


def test_interpolate_rejects_non_positive_period() -> None:
    with pytest.raises(ValueError):
        interpolate(PREV, NEW, 0.0, 0.0)


def test_limit_step_passes_small_steps_untouched() -> None:
    cmd = [x + 0.001 for x in PREV]
    assert limit_step(cmd, PREV, 0.01) == pytest.approx(cmd)


def test_limit_step_caps_each_joint_independently_and_keeps_sign() -> None:
    cmd = [PREV[0] + 1.0, PREV[1] - 1.0, PREV[2], PREV[3] + 0.005, PREV[4], PREV[5]]
    out = limit_step(cmd, PREV, 0.01)
    assert out == pytest.approx([PREV[0] + 0.01, PREV[1] - 0.01, PREV[2], PREV[3] + 0.005, PREV[4], PREV[5]])


def test_step_of_pi_is_followed_without_any_increment_over_the_limit() -> None:
    """Escalón de π rad en una articulación a 125 Hz y 60°/s: cada ciclo avanza como mucho el límite."""
    max_step_rad = math.radians(60) / 125
    target = [math.pi, 0.0, 0.0, 0.0, 0.0, 0.0]
    last = [0.0] * 6
    for _ in range(1000):
        cmd = limit_step(target, last, max_step_rad)
        assert max(abs(c - lst) for c, lst in zip(cmd, last, strict=True)) <= max_step_rad + 1e-12
        last = cmd
    assert last == pytest.approx(target)
