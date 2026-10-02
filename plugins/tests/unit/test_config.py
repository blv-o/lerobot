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
