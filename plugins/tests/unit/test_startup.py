"""Confirmación del operador y comprobación de la posición inicial antes de conectar."""

import math
from collections.abc import Callable

import pytest
from ur_core.startup import confirm_and_check, pose_errors

START_RAD = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
TOL_RAD = math.radians(2)


def answers(*replies: str) -> Callable[[str], str]:
    it = iter(replies)
    return lambda _prompt: next(it)


def test_yes_with_all_checks_ok_returns_true() -> None:
    calls: list[int] = []

    def checks() -> list[str]:
        calls.append(1)
        return []

    assert confirm_and_check("¿Continuar?", checks, ask=answers("yes")) is True
    assert calls == [1]


def test_no_returns_false_without_running_checks() -> None:
    def checks() -> list[str]:
        raise AssertionError("con `no` no se comprueba nada")

    assert confirm_and_check("¿Continuar?", checks, ask=answers("no")) is False


def test_other_answers_ask_again() -> None:
    assert confirm_and_check("¿Continuar?", lambda: [], ask=answers("", "si", "y", " YES ")) is True


def test_failed_checks_are_shown_and_question_repeated(capsys: pytest.CaptureFixture[str]) -> None:
    results = iter([["follower wrist_2: 5,3° fuera de tolerancia"], []])
    assert confirm_and_check("¿Continuar?", lambda: next(results), ask=answers("yes", "yes")) is True
    assert "follower wrist_2: 5,3° fuera de tolerancia" in capsys.readouterr().out


def test_failed_checks_then_no_returns_false() -> None:
    assert confirm_and_check("¿Continuar?", lambda: ["algo falla"], ask=answers("yes", "no")) is False


def test_pose_within_tolerance_has_no_errors() -> None:
    q = [x + math.radians(1.9) for x in START_RAD]
    assert pose_errors("follower", q, START_RAD, TOL_RAD) == []


def test_pose_error_names_robot_joint_and_degrees_with_decimal_comma() -> None:
    q = list(START_RAD)
    q[4] += math.radians(5.3)
    q[0] -= math.radians(3)
    assert pose_errors("follower", q, START_RAD, TOL_RAD) == [
        "follower base: 3,0° fuera de tolerancia",
        "follower wrist_2: 5,3° fuera de tolerancia",
    ]
