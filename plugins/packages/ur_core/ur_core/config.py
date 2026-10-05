"""Carga del YAML compartido por `ur_follower` y `ur_leader`.

El YAML usa las unidades cómodas para una persona (grados, ms); el resto del código solo ve
estas dataclasses inmutables en rad y s, para que ninguna conversión se repita ni se olvide.
No hay valores por defecto: la plantilla `plugins/configs/ur_config.yaml` lleva todas las
claves y lo que se ejecuta es exactamente lo que está escrito en el fichero.
"""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

PROFILES = ("sim", "real")
# Todos se controlan igual por RTDE (6 articulaciones, servoj): un modelo nuevo se añade aquí.
UR_TYPES = ("ur3e", "ur5e", "ur7e", "ur10e", "ur12e", "ur15", "ur16e", "ur20", "ur30")
# Réplicas de bajo coste con la cinemática del UR, leídas por un puerto serie; solo como leader.
LOW_COST_LEADER_TYPES = ("feetech", "as5600")
DIRECTIONS = ("normal", "invertido")
GAIN_RANGE = (100, 2000)
LOOKAHEAD_RANGE_S = (0.03, 0.2)
N_JOINTS = 6
# Orden del controlador UR (el mismo que `actual_q`), usado en todo el paquete.
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist_1", "wrist_2", "wrist_3")
TEMPLATE = "plugins/configs/ur_config.yaml"

# Claves que debe tener el YAML (las hojas valen None; solo importa la forma).
# La sección `leader` depende de su `type`: ver `_leader_schema`.
UR_LEADER_SCHEMA: dict[str, Any] = {"type": None, "ip": None, "rtde_hz": None, "timeout_ms": None}
LOW_COST_LEADER_SCHEMA: dict[str, Any] = {
    "type": None,
    "port": None,
    "hz": None,
    "timeout_ms": None,
    "mapping": {"direction": None},
}
SCHEMA: dict[str, Any] = {
    "leader": UR_LEADER_SCHEMA,
    "follower": {
        "type": None,
        "ip": None,
        "servo": {
            "hz": None,
            "gain": None,
            "lookahead_s": None,
            "max_joint_speed_deg_s": None,
            "target_period_ms": None,
        },
        "watchdog": {"hold_ms": None, "stop_ms": None},
    },
    "start_pose_deg": None,
    "start_tolerance_deg": None,
}


class ConfigError(ValueError):
    """Error de configuración que señala el campo exacto, para corregir el YAML sin adivinar."""

    def __init__(self, field: str, reason: str) -> None:
        super().__init__(f"{field}: {reason}")
        self.field = field
        self.reason = reason


@dataclass(frozen=True)
class UrLeaderConfig:
    type: str
    ip: str
    rtde_hz: float
    timeout_s: float


@dataclass(frozen=True)
class LowCostLeaderConfig:
    type: str
    port: str
    hz: float
    timeout_s: float
    direction: tuple[str, ...]  # "normal" | "invertido" por articulación, en el orden base..wrist_3


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
    leader: UrLeaderConfig | LowCostLeaderConfig
    follower: FollowerConfig
    start_pose_rad: tuple[float, ...]
    start_tolerance_rad: float


def load_config(path: Path) -> TeleopConfig:
    resolved = _read_yaml(path)
    if "profile" not in resolved:
        raise ConfigError("profile", f"obligatoria (ver la plantilla {TEMPLATE})")
    leader_profile, follower_profile = _parse_profile(resolved.pop("profile"))
    # start_pose_deg/start_tolerance_deg no dependen del perfil: se dejan tal cual.
    if "leader" in resolved:
        resolved["leader"] = _resolve(resolved["leader"], leader_profile)
    if "follower" in resolved:
        resolved["follower"] = _resolve(resolved["follower"], follower_profile)
    _check_keys(resolved, {**SCHEMA, "leader": _leader_schema(resolved.get("leader"))}, "")
    _validate(resolved)
    return _build(resolved)


def _leader_schema(leader: Any) -> dict[str, Any]:
    """Elige las claves de `leader` según su tipo: un UR se lee por RTDE, una réplica por serie."""
    if not isinstance(leader, dict):
        return UR_LEADER_SCHEMA  # _check_keys informará de que falta la sección o no lo es
    if "type" not in leader:
        raise ConfigError("leader.type", f"obligatoria (ver la plantilla {TEMPLATE})")
    leader_type = leader["type"]
    if isinstance(leader_type, dict):
        raise ConfigError("leader.type", "un valor por perfil debe ser {sim: ..., real: ...} completo")
    if leader_type in LOW_COST_LEADER_TYPES:
        return LOW_COST_LEADER_SCHEMA
    if leader_type in UR_TYPES:
        return UR_LEADER_SCHEMA
    valid = list(UR_TYPES + LOW_COST_LEADER_TYPES)
    raise ConfigError("leader.type", f"tipo {leader_type!r} no válido; tipos válidos: {valid}")


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
        return (
            _check_profile("profile.leader", profile["leader"]),
            _check_profile("profile.follower", profile["follower"]),
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


def _check_keys(user: dict[str, Any], schema: dict[str, Any], path: str) -> None:
    """Exige exactamente las claves de SCHEMA con su forma.

    Así un YAML mal escrito acaba siempre en un ConfigError con el campo, nunca en un KeyError
    o TypeError más adelante.
    """
    unknown = sorted(user.keys() - schema.keys(), key=str)
    if unknown:
        raise ConfigError(f"{path}{unknown[0]}", "clave desconocida")
    for key, sub_schema in schema.items():
        field = f"{path}{key}"
        if key not in user:
            raise ConfigError(field, f"obligatoria (ver la plantilla {TEMPLATE})")
        if isinstance(sub_schema, dict):
            if not isinstance(user[key], dict):
                raise ConfigError(field, "debe ser una sección con sus claves")
            _check_keys(user[key], sub_schema, f"{field}.")
        elif isinstance(user[key], dict):
            raise ConfigError(field, "un valor por perfil debe ser {sim: ..., real: ...} completo")


def _validate(c: dict[str, Any]) -> None:
    follower = c["follower"]
    servo, watchdog = follower["servo"], follower["watchdog"]
    _check_leader(c["leader"])
    if follower["type"] not in UR_TYPES:
        raise ConfigError(
            "follower.type", f"tipo {follower['type']!r} no válido; tipos válidos: {list(UR_TYPES)}"
        )
    _check_text("follower.ip", follower["ip"])
    _check_positive("follower.servo.hz", servo["hz"])
    _check_range("follower.servo.gain", servo["gain"], GAIN_RANGE)
    _check_range("follower.servo.lookahead_s", servo["lookahead_s"], LOOKAHEAD_RANGE_S)
    _check_number("follower.servo.max_joint_speed_deg_s", servo["max_joint_speed_deg_s"])
    _check_number("follower.servo.target_period_ms", servo["target_period_ms"])
    _check_watchdog(watchdog)
    _check_start_pose(c["start_pose_deg"])
    _check_number("start_tolerance_deg", c["start_tolerance_deg"])


def _check_leader(leader: dict[str, Any]) -> None:
    # El tipo ya se validó en _leader_schema.
    _check_number("leader.timeout_ms", leader["timeout_ms"])
    if leader["type"] in UR_TYPES:
        _check_text("leader.ip", leader["ip"])
        _check_positive("leader.rtde_hz", leader["rtde_hz"])
        return
    _check_text("leader.port", leader["port"])
    _check_positive("leader.hz", leader["hz"])
    direction = leader["mapping"]["direction"]
    if (
        not isinstance(direction, list)
        or len(direction) != N_JOINTS
        or any(d not in DIRECTIONS for d in direction)
    ):
        raise ConfigError(
            "leader.mapping.direction",
            f"debe ser una lista de {N_JOINTS} valores de {list(DIRECTIONS)} (vale {direction!r})",
        )


def _check_text(field: str, value: Any) -> None:
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


def _build_leader(leader: dict[str, Any]) -> UrLeaderConfig | LowCostLeaderConfig:
    timeout_s = leader["timeout_ms"] / 1000
    if leader["type"] in UR_TYPES:
        return UrLeaderConfig(
            type=leader["type"], ip=leader["ip"], rtde_hz=leader["rtde_hz"], timeout_s=timeout_s
        )
    return LowCostLeaderConfig(
        type=leader["type"],
        port=leader["port"],
        hz=leader["hz"],
        timeout_s=timeout_s,
        direction=tuple(leader["mapping"]["direction"]),
    )


def _build(c: dict[str, Any]) -> TeleopConfig:
    follower = c["follower"]
    servo, watchdog = follower["servo"], follower["watchdog"]
    return TeleopConfig(
        leader=_build_leader(c["leader"]),
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
