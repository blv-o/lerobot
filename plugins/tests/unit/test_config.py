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
        "timeout_ms": 100,
    },
    "follower": {
        "type": "ur3e",
        "ip": {"sim": FOLLOWER_SIM_IP, "real": FOLLOWER_REAL_IP},
    },
    "start_pose_deg": [0, -90, 90, -90, -90, 0],
    "start_tolerance_deg": 2,
}

# Todas las hojas de BASE: cada una es obligatoria en el YAML.
LEAF_KEYS = [
    "profile",
    "leader.type",
    "leader.ip",
    "leader.timeout_ms",
    "follower.type",
    "follower.ip",
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
    data["leader"]["timeout_ms"] = {"sim": 100, "real": 200}
    assert load(tmp_path, data).leader.timeout_s == pytest.approx(0.2)


def test_values_are_converted_to_radians_and_seconds(tmp_path: Path) -> None:
    config = load(tmp_path, base())
    assert config.start_pose_rad == pytest.approx(
        (0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0)
    )
    assert config.start_tolerance_rad == pytest.approx(math.radians(2))
    assert config.leader.timeout_s == pytest.approx(0.1)


def test_config_is_immutable(tmp_path: Path) -> None:
    config = load(tmp_path, base())
    assert isinstance(config.start_pose_rad, tuple)
    with pytest.raises(dataclasses.FrozenInstanceError):
        config.follower.ip = "10.0.0.5"  # type: ignore[misc]
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


def case(case_id: str, dotted: str, value: Any, make: Any = base, reason: str = "") -> Any:
    """Un YAML (`make()` con `dotted` = `value`) que se rechaza con ConfigError en ese campo."""
    return pytest.param(make, dotted, value, reason, id=case_id)


REJECTED = [
    # Una clave de leader UR en un leader low-cost no se ignora: el YAML estaría mezclando tipos.
    case("low_cost-ur_ip", "leader.ip", "127.0.0.3", make=low_cost_base),
    case("low_cost-hz_0", "leader.hz", 0, make=low_cost_base),
    case("low_cost-port_number", "leader.port", 3, make=low_cost_base),
    case("low_cost-direction_5", "leader.mapping.direction", ["normal"] * 5, make=low_cost_base),
    case(
        "low_cost-direction_inverted",
        "leader.mapping.direction",
        ["normal"] * 5 + ["inverted"],
        make=low_cost_base,
    ),
    # Una clave que el código no usa no se ignora en silencio. Velocidad e interpolación las fija
    # solo el programa del TP: en el YAML no harían nada.
    case("unknown-gripper_follower", "gripper_follower", {"type": "none"}),
    case("unknown-mapping", "mapping", {"direction": ["normal"] * 6}),
    case("unknown-leader.port", "leader.port", "COM3"),
    case("unknown-leader.mapping", "leader.mapping", {"direction": ["normal"] * 6}),
    case("unknown-leader.hz", "leader.hz", 100),
    case("unknown-leader.rtde_hz", "leader.rtde_hz", 500),  # el leader UR se lee al ritmo fijo del código
    case("unknown-follower.servo", "follower.servo", {"hz": 125}),
    case("unknown-follower.watchdog", "follower.watchdog", {"hz": 125}),
    case("start_pose-5_joints", "start_pose_deg", [0, -90, 90, -90, -90]),
    case("start_pose-7_joints", "start_pose_deg", [0, -90, 90, -90, -90, 0, 0]),
    case("start_pose-text", "start_pose_deg", [0, -90, 90, -90, -90, "a"]),
    # Con tolerancia ≤ 0 ningún robot pasaría el arranque y con `timeout_ms` ≤ 0 el leader
    # fallaría en cada lectura.
    case("tolerance-negative", "start_tolerance_deg", -1),
    case("tolerance-zero", "start_tolerance_deg", 0),
    case("timeout-negative", "leader.timeout_ms", -1),
    case("timeout-zero", "leader.timeout_ms", 0),
    # Con una tolerancia grande, los dos robots arrancarían lejos entre sí y la primera consigna
    # pediría un salto.
    case("tolerance-over_5", "start_tolerance_deg", 5.1),
    case("tolerance-30", "start_tolerance_deg", 30),
    # `.nan`/`.inf` en el YAML son floats válidos para Python y pasarían las comparaciones.
    case("start_pose-nan", "start_pose_deg", [0, -90, float("nan"), -90, -90, 0], reason="finito"),
    case("start_pose-inf", "start_pose_deg", [0, -90, 90, -90, -90, float("-inf")], reason="finito"),
    case("timeout-inf", "leader.timeout_ms", float("inf"), reason="finito"),
    case("timeout-nan", "leader.timeout_ms", float("nan"), reason="finito"),
    case("tolerance-inf", "start_tolerance_deg", float("inf"), reason="finito"),
    case("tolerance-nan", "start_tolerance_deg", float("nan"), reason="finito"),
    case("leader_ip-number", "leader.ip", 1234),
    case("follower_ip-number", "follower.ip", 1234),
    # Un YAML mal formado da siempre ConfigError con el campo, nunca TypeError/KeyError.
    case("shape-leader_none", "leader", None),
    case("shape-type_sim_only", "leader.type", {"sim": "ur3e"}),
    case("shape-ip_extra_profile", "follower.ip", {"sim": "a", "real": "b", "extra": "c"}),
]


@pytest.mark.parametrize(("make", "dotted", "value", "reason"), REJECTED)
def test_invalid_yaml_is_rejected_naming_the_field(
    tmp_path: Path, make: Any, dotted: str, value: Any, reason: str
) -> None:
    data = make()
    set_path(data, dotted, value)
    with pytest.raises(ConfigError) as exc:
        load(tmp_path, data)
    assert exc.value.field == dotted
    assert reason in exc.value.reason


def test_start_tolerance_of_5_deg_is_accepted(tmp_path: Path) -> None:
    data = base()
    data["start_tolerance_deg"] = 5
    assert load(tmp_path, data).start_tolerance_rad == pytest.approx(math.radians(5))


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


@pytest.mark.parametrize("name", ["no_existe.yaml", "."])
def test_unreadable_yaml_names_its_absolute_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, name: str
) -> None:
    """La ruta la da el usuario, a menudo relativa: el error dice qué fichero buscó de verdad."""
    monkeypatch.chdir(tmp_path)
    with pytest.raises(ConfigError) as exc:
        load_config(Path(name))
    assert str((tmp_path / name).absolute()) in str(exc.value)


@pytest.mark.parametrize(
    ("content", "reason"),
    [
        ("leader: [ur3e\n".encode(), "YAML mal escrito"),
        ("# posición inicial\nprofile: sim\n".encode("cp1252"), "UTF-8"),
    ],
    ids=["bad_yaml", "not_utf8"],
)
def test_malformed_yaml_names_its_absolute_path_and_the_reason(
    tmp_path: Path, content: bytes, reason: str
) -> None:
    """Un YAML roto o guardado en otra codificación da ConfigError, no un error de pyyaml."""
    path = tmp_path / "ur_config.yaml"
    path.write_bytes(content)
    with pytest.raises(ConfigError) as exc:
        load_config(path)
    assert exc.value.field == str(path.absolute())
    assert reason in exc.value.reason


@pytest.mark.parametrize("content", ["", "- 1\n- 2\n", "solo texto\n"])
def test_root_must_be_a_mapping(tmp_path: Path, content: str) -> None:
    path = tmp_path / "ur_config.yaml"
    path.write_text(content, encoding="utf-8")
    with pytest.raises(ConfigError):
        load_config(path)
