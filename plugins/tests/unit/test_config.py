"""Carga, resolución de perfiles sim/real y validación del YAML compartido por los plugins UR."""

import copy
import dataclasses
import math
from pathlib import Path
from typing import Any

import pytest
import yaml
from ur_core import ConfigError, TeleopConfig, load_config
from ur_core.config import LOW_COST_LEADER_TYPES, UR_TYPES, LowCostLeaderConfig, UrLeaderConfig

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

# Todas las hojas de BASE: cada una es obligatoria en el YAML.
LEAF_KEYS = [
    "profile",
    "leader.type",
    "leader.ip",
    "leader.rtde_hz",
    "leader.timeout_ms",
    "follower.type",
    "follower.ip",
    "follower.servo.hz",
    "follower.servo.gain",
    "follower.servo.lookahead_s",
    "follower.servo.max_joint_speed_deg_s",
    "follower.servo.target_period_ms",
    "follower.watchdog.hold_ms",
    "follower.watchdog.stop_ms",
    "start_pose_deg",
    "start_tolerance_deg",
]


def load(tmp_path: Path, data: Any) -> TeleopConfig:
    path = tmp_path / "ur_config.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return load_config(path)


def base() -> dict[str, Any]:
    return copy.deepcopy(BASE)


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


# ---------- Carga y resolución de perfiles ----------


@pytest.mark.parametrize(
    ("profile", "leader_ip", "follower_ip"),
    [("sim", LEADER_SIM_IP, FOLLOWER_SIM_IP), ("real", LEADER_REAL_IP, FOLLOWER_REAL_IP)],
)
def test_single_profile_resolves_both_sides(
    tmp_path: Path, profile: str, leader_ip: str, follower_ip: str
) -> None:
    data = base()
    data["profile"] = profile
    config = load(tmp_path, data)
    assert config.leader.ip == leader_ip
    assert config.follower.ip == follower_ip


def test_mixed_profile_resolves_each_side_with_its_own(tmp_path: Path) -> None:
    """Leader real en freedrive con el follower en URSim."""
    data = base()
    data["profile"] = {"leader": "real", "follower": "sim"}
    config = load(tmp_path, data)
    assert config.leader.ip == LEADER_REAL_IP
    assert config.follower.ip == FOLLOWER_SIM_IP


@pytest.mark.parametrize("profile", ["sim", "real"])
def test_plain_field_has_same_value_in_both_profiles(tmp_path: Path, profile: str) -> None:
    data = base()
    data["profile"] = profile
    data["follower"]["ip"] = "10.0.0.5"
    assert load(tmp_path, data).follower.ip == "10.0.0.5"


def test_sim_real_form_works_in_nested_fields(tmp_path: Path) -> None:
    data = base()
    data["profile"] = "real"
    data["follower"]["servo"]["hz"] = {"sim": 125, "real": 500}
    assert load(tmp_path, data).follower.servo.hz == 500


def test_values_are_converted_to_radians_and_seconds(tmp_path: Path) -> None:
    config = load(tmp_path, base())
    assert config.start_pose_rad == pytest.approx(
        (0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0)
    )
    assert config.start_tolerance_rad == pytest.approx(math.radians(2))
    assert config.leader.rtde_hz == 500
    assert config.leader.timeout_s == pytest.approx(0.1)
    assert config.follower.servo.hz == 125
    assert config.follower.servo.gain == 300
    assert config.follower.servo.max_joint_speed_rad_s == pytest.approx(math.radians(60))
    assert config.follower.servo.target_period_s == pytest.approx(0.033)
    assert config.follower.servo.lookahead_s == pytest.approx(0.1)
    assert config.follower.watchdog.hold_s == pytest.approx(0.1)
    assert config.follower.watchdog.stop_s == pytest.approx(0.5)


def test_config_is_immutable(tmp_path: Path) -> None:
    config = load(tmp_path, base())
    assert isinstance(config.start_pose_rad, tuple)
    with pytest.raises(dataclasses.FrozenInstanceError):
        config.follower.servo.gain = 1000  # type: ignore[misc]
    with pytest.raises(dataclasses.FrozenInstanceError):
        config.leader = config.leader  # type: ignore[misc]


@pytest.mark.parametrize("model", UR_TYPES)
def test_every_ur_model_is_accepted(tmp_path: Path, model: str) -> None:
    data = base()
    data["leader"]["type"] = model
    data["follower"]["type"] = model
    config = load(tmp_path, data)
    assert config.leader.type == model
    assert config.follower.type == model


def test_reference_template_loads() -> None:
    """La plantilla que se pasa a `--robot.config_path`/`--teleop.config_path`."""
    template = Path(__file__).parents[2] / "configs" / "ur_config.yaml"
    config = load_config(template)
    assert config.leader.ip == LEADER_SIM_IP
    assert config.follower.ip == FOLLOWER_SIM_IP


# ---------- Validaciones ----------


@pytest.mark.parametrize("dotted", LEAF_KEYS)
def test_every_key_is_required(tmp_path: Path, dotted: str) -> None:
    """Sin valores por defecto: el YAML es la única fuente de la configuración."""
    data = base()
    remove_path(data, dotted)
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == dotted


def test_unknown_leader_type_lists_valid_types(tmp_path: Path) -> None:
    data = base()
    data["leader"]["type"] = "ur99"
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == "leader.type"
    assert all(t in exc.value.reason for t in UR_TYPES + LOW_COST_LEADER_TYPES)


@pytest.mark.parametrize("value", ["ur99", "feetech", "as5600"])
def test_follower_type_must_be_ur(tmp_path: Path, value: str) -> None:
    """El follower siempre es un UR: un tipo low-cost solo vale como leader."""
    data = base()
    data["follower"]["type"] = value
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == "follower.type"
    assert all(model in exc.value.reason for model in UR_TYPES)


def test_ur_leader_gives_ur_leader_config(tmp_path: Path) -> None:
    leader = load(tmp_path, base()).leader
    assert isinstance(leader, UrLeaderConfig)


# ---------- Leader low-cost (réplica con servos Feetech o encoders AS5600) ----------


def low_cost_base(leader_type: str = "feetech") -> dict[str, Any]:
    data = base()
    data["leader"] = {
        "type": leader_type,
        "port": {"sim": "COM3", "real": "COM4"},
        "hz": 100,
        "timeout_ms": 50,
        "mapping": {"direction": ["normal", "invertido", "normal", "normal", "invertido", "normal"]},
    }
    return data


@pytest.mark.parametrize("leader_type", LOW_COST_LEADER_TYPES)
def test_low_cost_leader_loads(tmp_path: Path, leader_type: str) -> None:
    leader = load(tmp_path, low_cost_base(leader_type)).leader
    assert isinstance(leader, LowCostLeaderConfig)
    assert leader.type == leader_type
    assert leader.port == "COM3"
    assert leader.hz == 100
    assert leader.timeout_s == pytest.approx(0.05)
    assert leader.direction == ("normal", "invertido", "normal", "normal", "invertido", "normal")


def test_low_cost_leader_port_follows_leader_profile(tmp_path: Path) -> None:
    data = low_cost_base()
    data["profile"] = {"leader": "real", "follower": "sim"}
    assert load(tmp_path, data).leader.port == "COM4"


@pytest.mark.parametrize(
    "dotted",
    [
        "leader.type",
        "leader.port",
        "leader.hz",
        "leader.timeout_ms",
        "leader.mapping",
        "leader.mapping.direction",
    ],
)
def test_low_cost_leader_keys_are_required(tmp_path: Path, dotted: str) -> None:
    data = low_cost_base()
    remove_path(data, dotted)
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == dotted


@pytest.mark.parametrize(("dotted", "value"), [("leader.ip", "127.0.0.3"), ("leader.rtde_hz", 500)])
def test_ur_only_keys_are_rejected_with_low_cost_leader(tmp_path: Path, dotted: str, value: Any) -> None:
    """Una clave de leader UR en un leader low-cost no se ignora: el YAML estaría mezclando tipos."""
    data = low_cost_base()
    set_path(data, dotted, value)
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == dotted


@pytest.mark.parametrize(
    ("dotted", "value"),
    [
        ("leader.hz", 0),
        ("leader.port", 3),
        ("leader.mapping.direction", ["normal"] * 5),
        ("leader.mapping.direction", ["normal"] * 5 + ["inverted"]),
    ],
)
def test_low_cost_leader_values_are_validated(tmp_path: Path, dotted: str, value: Any) -> None:
    data = low_cost_base()
    set_path(data, dotted, value)
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == dotted


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
    """Una clave que el código no usa no se ignora en silencio."""
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
    data = base()
    set_path(data, dotted, value)
    load(tmp_path, data)


@pytest.mark.parametrize(("hold_ms", "accepted"), [(33, False), (20, False), (34, True)])
def test_watchdog_hold_must_exceed_target_period(tmp_path: Path, hold_ms: int, accepted: bool) -> None:
    """Con hold_ms <= target_period_ms, el follower se quedaría quieto entre dos consignas normales."""
    data = base()
    set_path(data, "follower.watchdog.hold_ms", hold_ms)
    if accepted:
        load(tmp_path, data)
        return
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == "follower.watchdog.hold_ms"


@pytest.mark.parametrize(("stop_ms", "accepted"), [(100, False), (101, True)])
def test_watchdog_stop_must_exceed_servo_period(tmp_path: Path, stop_ms: int, accepted: bool) -> None:
    """Con stop_ms <= 1000 / hz, el watchdog saltaría entre dos ciclos RTDE normales.

    Con hz=125 el límite (8 ms) queda por debajo de hold_ms: se baja hz para aislar esta regla.
    """
    data = base()
    set_path(data, "follower.servo.hz", 10)
    set_path(data, "follower.watchdog.hold_ms", 50)
    set_path(data, "follower.watchdog.stop_ms", stop_ms)
    if accepted:
        load(tmp_path, data)
        return
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == "follower.watchdog.stop_ms"


@pytest.mark.parametrize(
    ("dotted", "value"),
    [
        ("follower.servo.max_joint_speed_deg_s", -60),
        ("follower.servo.max_joint_speed_deg_s", 0),
        ("follower.servo.target_period_ms", -1),
        ("follower.servo.target_period_ms", 0),
        ("start_tolerance_deg", -1),
        ("start_tolerance_deg", 0),
        ("leader.timeout_ms", -1),
        ("leader.timeout_ms", 0),
    ],
)
def test_non_positive_value_is_rejected(tmp_path: Path, dotted: str, value: Any) -> None:
    """Una velocidad negativa haría derivar el follower sin parar y un periodo 0 rompe el bucle."""
    data = base()
    set_path(data, dotted, value)
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == dotted


@pytest.mark.parametrize(
    ("dotted", "value"),
    [
        ("follower.servo.max_joint_speed_deg_s", float("nan")),
        ("follower.servo.max_joint_speed_deg_s", float("inf")),
        ("follower.servo.hz", float("nan")),
        ("follower.servo.hz", float("inf")),
        ("follower.servo.gain", float("nan")),
        ("follower.servo.gain", float("inf")),
        ("start_pose_deg", [0, -90, float("nan"), -90, -90, 0]),
        ("start_pose_deg", [0, -90, 90, -90, -90, float("-inf")]),
    ],
)
def test_non_finite_value_is_rejected(tmp_path: Path, dotted: str, value: Any) -> None:
    """`.nan`/`.inf` en el YAML son floats válidos para Python y pasarían las comparaciones."""
    data = base()
    set_path(data, dotted, value)
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == dotted
    assert "finito" in exc.value.reason


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
