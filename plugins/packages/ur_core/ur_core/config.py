"""Carga del YAML compartido por `ur_follower` y `ur_leader` (RF-1..7 de SPEC_003).

El YAML usa las unidades cómodas para una persona (grados, ms); el resto del código solo ve
estas dataclasses inmutables en rad y s, para que ninguna conversión se repita ni se olvide.
"""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

PROFILES = ("sim", "real")
UR_TYPES = ("ur3e", "ur5e")
GAIN_RANGE = (100, 2000)
LOOKAHEAD_RANGE_S = (0.03, 0.2)
N_JOINTS = 6

# Valores del YAML de la spec: una clave ausente toma este valor (decisión 2026-10-03).
# Las `ip` valen None: no hay IP correcta por defecto para un robot real, así que la validación
# las exige (la del leader solo si es un UR; un leader low-cost de SPEC_005 no tiene IP).
DEFAULT_PROFILE = "sim"
DEFAULTS: dict[str, Any] = {
    "leader": {"type": "ur3e", "ip": None, "rtde_hz": 500, "timeout_ms": 100},
    "follower": {
        "type": "ur3e",
        "ip": None,
        "servo": {
            "hz": 125,
            "gain": 300,
            "lookahead_s": 0.1,
            "max_joint_speed_deg_s": 60,
            "target_period_ms": 33,
        },
        "watchdog": {"hold_ms": 100, "stop_ms": 500},
    },
    "start_pose_deg": [0, -90, 90, -90, -90, 0],
    "start_tolerance_deg": 2,
}


class ConfigError(ValueError):
    """Error de configuración que señala el campo exacto, para corregir el YAML sin adivinar."""

    def __init__(self, field: str, reason: str) -> None:
        super().__init__(f"{field}: {reason}")
        self.field = field
        self.reason = reason


@dataclass(frozen=True)
class LeaderConfig:
    type: str
    ip: str | None  # None solo para leaders que no son UR (SPEC_005)
    rtde_hz: float
    timeout_s: float


@dataclass(frozen=True)
class ServoConfig:
    hz: float
    gain: float
    lookahead_s: float
    max_joint_speed_rad_s: float
    target_period_s: float


@dataclass(frozen=True)
class WatchdogConfig:
    hold_s: float
    stop_s: float


@dataclass(frozen=True)
class FollowerConfig:
    type: str
    ip: str
    servo: ServoConfig
    watchdog: WatchdogConfig


@dataclass(frozen=True)
class TeleopConfig:
    leader: LeaderConfig
    follower: FollowerConfig
    start_pose_rad: tuple[float, ...]
    start_tolerance_rad: float


def load_config(path: Path) -> TeleopConfig:
    raw = _read_yaml(path)
    resolved = dict(raw)
    leader_profile, follower_profile = _parse_profile(resolved.pop("profile", DEFAULT_PROFILE))
    if "leader" in raw:
        resolved["leader"] = _resolve(raw["leader"], leader_profile)
    if "follower" in raw:
        resolved["follower"] = _resolve(raw["follower"], follower_profile)
    merged = _merge_defaults(resolved, DEFAULTS, "")
    _validate(merged)
    return _build(merged)


def _read_yaml(path: Path) -> dict[str, Any]:
    # utf-8 explícito: en Windows el defecto es cp1252 y el YAML lleva comentarios en español.
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise ConfigError("<raíz>", "el YAML debe ser un diccionario de claves")
    return raw


def _parse_profile(profile: Any) -> tuple[str, str]:
    """Devuelve (perfil del leader, perfil del follower)."""
    if isinstance(profile, dict):
        if set(profile) != {"leader", "follower"}:
            raise ConfigError("profile", "como diccionario debe tener exactamente leader y follower")
        return _check_profile("profile.leader", profile["leader"]), _check_profile(
            "profile.follower", profile["follower"]
        )
    checked = _check_profile("profile", profile)
    return checked, checked


def _check_profile(field: str, value: Any) -> str:
    if value not in PROFILES:
        raise ConfigError(field, f"debe ser uno de {list(PROFILES)} (vale {value!r})")
    return value


def _resolve(node: Any, profile: str) -> Any:
    """Sustituye cada `{sim: x, real: y}` por la rama del perfil, a cualquier profundidad."""
    if isinstance(node, dict):
        if set(node) == set(PROFILES):
            return node[profile]
        return {key: _resolve(value, profile) for key, value in node.items()}
    return node


def _merge_defaults(user: dict[str, Any], defaults: dict[str, Any], path: str) -> dict[str, Any]:
    """Completa con DEFAULTS y rechaza lo que no encaja con su forma, para que un YAML mal
    escrito acabe siempre en un ConfigError con el campo y nunca en un TypeError más adelante."""
    unknown = sorted(user.keys() - defaults.keys(), key=str)
    if unknown:
        raise ConfigError(f"{path}{unknown[0]}", "clave desconocida en SPEC_003")
    merged: dict[str, Any] = {}
    for key, default in defaults.items():
        field = f"{path}{key}"
        if key not in user:
            merged[key] = default
        elif isinstance(default, dict):
            if not isinstance(user[key], dict):
                raise ConfigError(field, "debe ser una sección con sus claves")
            merged[key] = _merge_defaults(user[key], default, f"{field}.")
        elif isinstance(user[key], dict):
            raise ConfigError(field, "un valor por perfil debe ser {sim: ..., real: ...} completo")
        else:
            merged[key] = user[key]
    return merged


def _validate(c: dict[str, Any]) -> None:
    leader, follower = c["leader"], c["follower"]
    servo, watchdog = follower["servo"], follower["watchdog"]
    _check_type("leader.type", leader["type"])
    _check_type("follower.type", follower["type"])
    if leader["type"] in UR_TYPES:
        _check_ip("leader.ip", leader["ip"], "obligatoria con leader UR")
    _check_ip("follower.ip", follower["ip"], "obligatoria")
    _check_positive("leader.rtde_hz", leader["rtde_hz"])
    _check_number("leader.timeout_ms", leader["timeout_ms"])
    _check_positive("follower.servo.hz", servo["hz"])
    _check_range("follower.servo.gain", servo["gain"], GAIN_RANGE)
    _check_range("follower.servo.lookahead_s", servo["lookahead_s"], LOOKAHEAD_RANGE_S)
    _check_number("follower.servo.max_joint_speed_deg_s", servo["max_joint_speed_deg_s"])
    _check_number("follower.servo.target_period_ms", servo["target_period_ms"])
    _check_watchdog(watchdog)
    _check_start_pose(c["start_pose_deg"])
    _check_number("start_tolerance_deg", c["start_tolerance_deg"])


def _check_type(field: str, value: Any) -> None:
    if value not in UR_TYPES:
        raise ConfigError(field, f"tipo {value!r} no válido; tipos válidos: {list(UR_TYPES)}")


def _check_ip(field: str, value: Any, missing_reason: str) -> None:
    if value is None:
        raise ConfigError(field, missing_reason)
    if not isinstance(value, str):
        raise ConfigError(field, f"debe ser texto (vale {value!r})")


def _check_number(field: str, value: Any) -> None:
    # bool es subclase de int en Python, pero `gain: true` es un error del YAML, no un 1.
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ConfigError(field, f"debe ser un número (vale {value!r})")


def _check_positive(field: str, value: Any) -> None:
    _check_number(field, value)
    if value <= 0:
        raise ConfigError(field, f"debe ser > 0 (vale {value})")


def _check_range(field: str, value: Any, limits: tuple[float, float]) -> None:
    _check_number(field, value)
    low, high = limits
    if not low <= value <= high:
        raise ConfigError(field, f"fuera de [{low}, {high}] (vale {value})")


def _check_watchdog(watchdog: dict[str, Any]) -> None:
    _check_positive("follower.watchdog.hold_ms", watchdog["hold_ms"])
    _check_positive("follower.watchdog.stop_ms", watchdog["stop_ms"])
    if watchdog["hold_ms"] >= watchdog["stop_ms"]:
        raise ConfigError(
            "follower.watchdog.hold_ms",
            f"debe ser < stop_ms ({watchdog['hold_ms']} >= {watchdog['stop_ms']})",
        )


def _check_start_pose(pose: Any) -> None:
    if not isinstance(pose, list) or len(pose) != N_JOINTS:
        raise ConfigError("start_pose_deg", f"debe ser una lista de {N_JOINTS} ángulos (vale {pose!r})")
    for i, value in enumerate(pose):
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ConfigError("start_pose_deg", f"el valor {i} no es un número ({value!r})")


def _build(c: dict[str, Any]) -> TeleopConfig:
    leader, follower = c["leader"], c["follower"]
    servo, watchdog = follower["servo"], follower["watchdog"]
    return TeleopConfig(
        leader=LeaderConfig(
            type=leader["type"],
            ip=leader["ip"],
            rtde_hz=leader["rtde_hz"],
            timeout_s=leader["timeout_ms"] / 1000,
        ),
        follower=FollowerConfig(
            type=follower["type"],
            ip=follower["ip"],
            servo=ServoConfig(
                hz=servo["hz"],
                gain=servo["gain"],
                lookahead_s=servo["lookahead_s"],
                max_joint_speed_rad_s=math.radians(servo["max_joint_speed_deg_s"]),
                target_period_s=servo["target_period_ms"] / 1000,
            ),
            watchdog=WatchdogConfig(
                hold_s=watchdog["hold_ms"] / 1000,
                stop_s=watchdog["stop_ms"] / 1000,
            ),
        ),
        start_pose_rad=tuple(math.radians(d) for d in c["start_pose_deg"]),
        start_tolerance_rad=math.radians(c["start_tolerance_deg"]),
    )
