"""F1: carga, resolución de perfiles y validación del YAML compartido (RF-1..7 de SPEC_003)."""

import copy
import dataclasses
import math
from pathlib import Path
from typing import Any

import pytest
import yaml

from ur_core import ConfigError, TeleopConfig, load_config

LEADER_SIM_IP = "127.0.0.3"
LEADER_REAL_IP = "192.168.1.11"
FOLLOWER_SIM_IP = "127.0.0.2"
FOLLOWER_REAL_IP = "192.168.1.10"

BASE: dict[str, Any] = {
    "profile": "sim",
    "leader": {
        "type": "ur3e",
        "ip": {"sim": LEADER_SIM_IP, "real": LEADER_REAL_IP},
        "rtde_hz": 500,
        "timeout_ms": 100,
    },
    "follower": {
        "type": "ur3e",
        "ip": {"sim": FOLLOWER_SIM_IP, "real": FOLLOWER_REAL_IP},
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


def load(tmp_path: Path, data: Any) -> TeleopConfig:
    path = tmp_path / "ur_config.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return load_config(path)


def base() -> dict[str, Any]:
    return copy.deepcopy(BASE)


# ---------- T1.1: carga y resolución de perfiles (RF-1, 2, 3, 7) ----------


@pytest.mark.parametrize(
    ("profile", "leader_ip", "follower_ip"),
    [("sim", LEADER_SIM_IP, FOLLOWER_SIM_IP), ("real", LEADER_REAL_IP, FOLLOWER_REAL_IP)],
)
def test_single_profile_resolves_both_sides(
    tmp_path: Path, profile: str, leader_ip: str, follower_ip: str
) -> None:
    """RF-1 (CA1.1)."""
    data = base()
    data["profile"] = profile
    config = load(tmp_path, data)
    assert config.leader.ip == leader_ip
    assert config.follower.ip == follower_ip


def test_mixed_profile_resolves_each_side_with_its_own(tmp_path: Path) -> None:
    """RF-2 (CA1.2): leader real en freedrive, follower en URSim (fase F4)."""
    data = base()
    data["profile"] = {"leader": "real", "follower": "sim"}
    config = load(tmp_path, data)
    assert config.leader.ip == LEADER_REAL_IP
    assert config.follower.ip == FOLLOWER_SIM_IP


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_plain_field_has_same_value_in_both_profiles(tmp_path: Path, profile: str) -> None:
    """RF-3 (CA1.3)."""
    data = base()
    data["profile"] = profile
    data["follower"]["ip"] = "10.0.0.5"
    assert load(tmp_path, data).follower.ip == "10.0.0.5"


def test_sim_real_form_works_in_nested_fields(tmp_path: Path) -> None:
    """RF-3: cualquier campo puede escribirse como {sim, real}, también dentro de `servo`."""
    data = base()
    data["profile"] = "real"
    data["follower"]["servo"]["hz"] = {"sim": 125, "real": 500}
    assert load(tmp_path, data).follower.servo.hz == 500


def test_values_are_converted_to_radians_and_seconds(tmp_path: Path) -> None:
    """RF-7 (CA1.9)."""
    config = load(tmp_path, base())
    assert config.start_pose_rad == pytest.approx(
        (0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0)
    )
    assert config.start_tolerance_rad == pytest.approx(math.radians(2))
    assert config.leader.timeout_s == pytest.approx(0.1)
    assert config.follower.servo.max_joint_speed_rad_s == pytest.approx(math.radians(60))
    assert config.follower.servo.target_period_s == pytest.approx(0.033)
    assert config.follower.servo.lookahead_s == pytest.approx(0.1)
    assert config.follower.watchdog.hold_s == pytest.approx(0.1)
    assert config.follower.watchdog.stop_s == pytest.approx(0.5)


def test_config_is_immutable(tmp_path: Path) -> None:
    """RF-7: las dataclasses no se pueden modificar tras cargar."""
    config = load(tmp_path, base())
    assert isinstance(config.start_pose_rad, tuple)
    with pytest.raises(dataclasses.FrozenInstanceError):
        config.follower.servo.gain = 1000  # type: ignore[misc]
    with pytest.raises(dataclasses.FrozenInstanceError):
        config.leader = config.leader  # type: ignore[misc]


def test_missing_keys_take_the_spec_defaults(tmp_path: Path) -> None:
    """Decisión 2026-10-03: con solo las IPs, el resto toma los valores del YAML de la spec."""
    config = load(
        tmp_path,
        {"leader": {"ip": LEADER_SIM_IP}, "follower": {"ip": FOLLOWER_SIM_IP}},
    )
    assert config.leader.type == "ur3e"
    assert config.leader.ip == LEADER_SIM_IP
    assert config.leader.rtde_hz == 500
    assert config.leader.timeout_s == pytest.approx(0.1)
    assert config.follower.type == "ur3e"
    assert config.follower.servo.hz == 125
    assert config.follower.servo.gain == 300
    assert config.follower.servo.lookahead_s == pytest.approx(0.1)
    assert config.follower.servo.max_joint_speed_rad_s == pytest.approx(math.radians(60))
    assert config.follower.servo.target_period_s == pytest.approx(0.033)
    assert config.follower.watchdog.hold_s == pytest.approx(0.1)
    assert config.follower.watchdog.stop_s == pytest.approx(0.5)
    assert config.start_pose_rad == pytest.approx(
        tuple(math.radians(d) for d in (0, -90, 90, -90, -90, 0))
    )
    assert config.start_tolerance_rad == pytest.approx(math.radians(2))


# ---------- T1.2: validaciones (RF-4, 5, 6) ----------


def set_path(data: dict[str, Any], dotted: str, value: Any) -> None:
    *parents, last = dotted.split(".")
    for key in parents:
        data = data[key]
    data[last] = value


def remove_path(data: dict[str, Any], dotted: str) -> None:
    *parents, last = dotted.split(".")
    for key in parents:
        data = data[key]
    data.pop(last)


@pytest.mark.parametrize("side", ["leader", "follower"])
def test_unknown_type_lists_valid_types(tmp_path: Path, side: str) -> None:
    """RF-4 (CA1.4)."""
    data = base()
    data[side]["type"] = "ur10e"
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == f"{side}.type"
    assert "ur3e" in exc.value.reason and "ur5e" in exc.value.reason


@pytest.mark.parametrize(
    ("dotted", "value"),
    [
        ("gripper_follower", {"type": "none"}),
        ("mapping", {"direction": ["normal"] * 6}),
        ("leader.port", "COM3"),
        ("follower.servo.foo", 1),
    ],
)
def test_unknown_key_is_rejected(tmp_path: Path, dotted: str, value: Any) -> None:
    """RF-5: una clave fuera de la spec (p. ej. de SPEC_004/005) no se ignora en silencio."""
    data = base()
    set_path(data, dotted, value)
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == dotted


@pytest.mark.parametrize(
    ("dotted", "value"),
    [
        ("follower.servo.gain", 99),
        ("follower.servo.gain", 2001),
        ("follower.servo.lookahead_s", 0.02),
        ("follower.servo.lookahead_s", 0.21),
        ("follower.servo.hz", 0),
        ("follower.servo.hz", -125),
        ("leader.rtde_hz", 0),
        ("follower.watchdog.hold_ms", 0),
        ("follower.watchdog.hold_ms", -1),
        ("follower.watchdog.hold_ms", 500),
        ("follower.watchdog.hold_ms", 600),
        ("start_pose_deg", [0, -90, 90, -90, -90]),
        ("start_pose_deg", [0, -90, 90, -90, -90, 0, 0]),
        ("start_pose_deg", [0, -90, 90, -90, -90, "a"]),
        ("follower.servo.gain", True),
        ("follower.servo.gain", "300"),
    ],
)
def test_out_of_range_value_is_rejected(tmp_path: Path, dotted: str, value: Any) -> None:
    """RF-6 (CA1.8)."""
    data = base()
    set_path(data, dotted, value)
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == dotted


@pytest.mark.parametrize(
    ("dotted", "value"),
    [
        ("follower.servo.gain", 100),
        ("follower.servo.gain", 2000),
        ("follower.servo.lookahead_s", 0.03),
        ("follower.servo.lookahead_s", 0.2),
        ("follower.watchdog.hold_ms", 499),
    ],
)
def test_range_limits_are_inclusive(tmp_path: Path, dotted: str, value: Any) -> None:
    """RF-6: los bordes de los intervalos cerrados son válidos."""
    data = base()
    set_path(data, dotted, value)
    load(tmp_path, data)


def test_follower_ip_is_required(tmp_path: Path) -> None:
    data = base()
    remove_path(data, "follower.ip")
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == "follower.ip"


def test_ur_leader_requires_ip(tmp_path: Path) -> None:
    """La IP del leader solo es obligatoria si es un UR (un leader low-cost de SPEC_005 no tiene)."""
    data = base()
    remove_path(data, "leader.ip")
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == "leader.ip"
    assert "UR" in exc.value.reason


@pytest.mark.parametrize("side", ["leader", "follower"])
def test_ip_must_be_text(tmp_path: Path, side: str) -> None:
    data = base()
    data[side]["ip"] = 1234
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == f"{side}.ip"


@pytest.mark.parametrize(
    "profile",
    ["simulation", {"leader": "real"}, {"leader": "real", "follower": "sim", "x": "sim"}, 1],
)
def test_invalid_profile_is_rejected(tmp_path: Path, profile: Any) -> None:
    data = base()
    data["profile"] = profile
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field.startswith("profile")


@pytest.mark.parametrize(
    ("dotted", "value"),
    [
        ("leader", None),
        ("follower.servo", [125, 300]),
        ("leader.type", {"sim": "ur3e"}),
        ("follower.ip", {"sim": "a", "real": "b", "extra": "c"}),
    ],
)
def test_wrong_shape_raises_config_error(tmp_path: Path, dotted: str, value: Any) -> None:
    """Un YAML mal formado da siempre ConfigError con el campo, nunca TypeError/KeyError."""
    data = base()
    set_path(data, dotted, value)
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == dotted


@pytest.mark.parametrize("content", ["", "- 1\n- 2\n", "solo texto\n"])
def test_root_must_be_a_mapping(tmp_path: Path, content: str) -> None:
    path = tmp_path / "ur_config.yaml"
    path.write_text(content, encoding="utf-8")
    with pytest.raises(ConfigError):
        load_config(path)
