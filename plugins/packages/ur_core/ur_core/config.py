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

# Valores del YAML de la spec: una clave ausente toma este valor (decisión 2026-10-03).
# `follower.ip` no tiene defecto: no hay IP correcta para un robot real.
DEFAULTS: dict[str, Any] = {
    "profile": "sim",
    "leader": {"type": "ur3e", "ip": None, "rtde_hz": 500, "timeout_ms": 100},
    "follower": {
        "type": "ur3e",
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
    leader_profile, follower_profile = _parse_profile(raw.get("profile", DEFAULTS["profile"]))
    resolved = dict(raw)
    if "leader" in raw:
        resolved["leader"] = _resolve(raw["leader"], leader_profile)
    if "follower" in raw:
        resolved["follower"] = _resolve(raw["follower"], follower_profile)
    merged = _merge_defaults(resolved, DEFAULTS, "")
    return _build(merged)


def _read_yaml(path: Path) -> dict[str, Any]:
    # utf-8 explícito: en Windows el defecto es cp1252 y el YAML lleva comentarios en español.
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _parse_profile(profile: Any) -> tuple[str, str]:
    if isinstance(profile, dict):
        return profile["leader"], profile["follower"]
    return profile, profile


def _resolve(node: Any, profile: str) -> Any:
    """Sustituye cada `{sim: x, real: y}` por la rama del perfil, a cualquier profundidad."""
    if isinstance(node, dict):
        if set(node) == set(PROFILES):
            return node[profile]
        return {key: _resolve(value, profile) for key, value in node.items()}
    return node


def _merge_defaults(user: dict[str, Any], defaults: dict[str, Any], path: str) -> dict[str, Any]:
    merged: dict[str, Any] = {}
    for key, default in defaults.items():
        field = f"{path}{key}"
        if key not in user:
            merged[key] = default
        elif isinstance(default, dict):
            merged[key] = _merge_defaults(user[key], default, f"{field}.")
        else:
            merged[key] = user[key]
    for key in user.keys() - defaults.keys():
        merged[key] = user[key]
    return merged


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
