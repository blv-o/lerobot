This file provides guidance to AI agents when working with code in this repository.

> **User-facing help → [`AGENT_GUIDE.md`](./AGENT_GUIDE.md)** (SO-101 setup, recording, picking a policy, training duration, eval — with copy-pasteable commands).

## Project Overview

LeRobot is a PyTorch-based library for real-world robotics, providing datasets, pretrained policies, and tools for training, evaluation, data collection, and robot control. It integrates with Hugging Face Hub for model/dataset sharing.

## Tech Stack

Python 3.12+ · PyTorch · Hugging Face (datasets, Hub, accelerate) · draccus (config/CLI) · Gymnasium (envs) · pip + conda (package management)

## Development Setup

This fork is installed with pip in editable mode inside the `lerobot` conda env (see `README.md`); there is no `uv.lock`.

```bash
python -m pip install -e ".[core_scripts,training,feetech]"   # Base + the extras used in this project
git lfs install && git lfs pull                               # Test artifacts
```

## Key Commands

```bash
pytest plugins/tests                       # UR plugin suite (own pytest.ini; add -m ursim with URSim up)
pytest tests -svv --maxfail=10             # Upstream LeRobot suite
ruff check . && ruff format --check .      # Lint + format (config in pyproject.toml)
```

## Architecture (`src/lerobot/`)

- **`scripts/`** — CLI entry points (`lerobot-train`, `lerobot-eval`, `lerobot-record`, etc.), mapped in `pyproject.toml [project.scripts]`.
- **`configs/`** — Dataclass configs parsed by draccus. `train.py` has `TrainPipelineConfig` (top-level). `policies.py` has `PreTrainedConfig` base. Polymorphism via `draccus.ChoiceRegistry` with `@register_subclass("name")` decorators.
- **`policies/`** — Each policy in its own subdir. All inherit `PreTrainedPolicy` (`nn.Module` + `HubMixin`) from `pretrained.py`. Factory with lazy imports in `factory.py`.
- **`processor/`** — Data transformation pipeline. `ProcessorStep` base with registry. `DataProcessorPipeline` / `PolicyProcessorPipeline` chain steps.
- **`datasets/`** — `LeRobotDataset` (episode-aware sampling + video decoding) and `LeRobotDatasetMetadata`.
- **`envs/`** — `EnvConfig` base in `configs.py`, factory in `factory.py`. Each env subclass defines `gym_kwargs` and `create_envs()`.
- **`robots/`, `motors/`, `cameras/`, `teleoperators/`** — Hardware abstraction layers.
- **`types.py`** and **`configs/types.py`** — Core type aliases and feature type definitions.

## Repository Structure (outside `src/`)

- **`plugins/`** — Own code of this fork (UR3e teleop over RTDE): `lerobot_robot_ur_follower`, `lerobot_teleoperator_ur_leader`, `ur_core`, URSim compose file and their tests. Each package has its own `pyproject.toml`.
- **`tests/`** — Upstream pytest suite organized by module. Fixtures in `tests/fixtures/`, mocks in `tests/mocks/`. Hardware tests use skip decorators from `tests/utils.py`.
- **Root files**: `pyproject.toml` (single source of truth for deps, build, tool config), `README.md` (what this fork is and how to install it).
- Upstream `docs/`, `examples/`, `docker/`, CI and community files were removed on purpose; see them in https://github.com/huggingface/lerobot.

## Notes

- **Mypy is gradual**: strict only for `lerobot.envs`, `lerobot.configs`, `lerobot.optim`, `lerobot.model`, `lerobot.cameras`, `lerobot.motors`, `lerobot.transport`. Add type annotations when modifying these modules.
- **Imports**: prefer top-level imports; relative (`from .sibling import X`) across sibling files within a module, absolute (`from lerobot.module import X`) across modules.
- **Optional dependencies**: many policies, envs, and robots are behind extras (e.g., `lerobot[aloha]`, see `pyproject.toml`). Guard optional imports with `TYPE_CHECKING or _foo_available` at module top + a `require_package(...)` check at use time. Reuse the `_foo_available` flags in `utils/import_utils.py`; don't call `is_package_available`.
- **Video decoding**: datasets can store observations as video files. `LeRobotDataset` handles frame extraction, but tests need ffmpeg installed.
- **Use the `lerobot` conda env's Python** (`python -m pip`, `python -m pytest`); never install into or edit `site-packages`.
