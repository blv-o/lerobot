# LeRobot para robots UR

Plug-in para robots UR sobre la libreria de [huggingface/lerobot](https://github.com/huggingface/lerobot).

Permite teleoperar un UR (follower) moviendo a mano otro UR en freedrive (leader), y usar los comandos de LeRobot (`lerobot-teleoperate`, `lerobot-record`…). La comunicación con los robots es por RTDE.

La librería `lerobot` (`src/`) es la del original; la documentación, ejemplos y CI se han retirado: [su documentación](https://huggingface.co/docs/lerobot).

En desarrollo la utilización de un leader low-cost en vez de un UR.

## Qué añade

| Dónde | Qué |
|---|---|
| `plugins/packages/lerobot_robot_ur_follower/` | Robot `ur_follower`: el UR que sigue las consignas |
| `plugins/packages/lerobot_teleoperator_ur_leader/` | Teleoperador `ur_leader`: el UR en freedrive del que se lee la posición articular |
| `plugins/packages/ur_core/` | Lógica común de ambos, con el cliente RTDE oficial de UR |
| `plugins/tp/follower_control.script` | Programa del follower para el Teach Pendant: se carga como nodo Script y se le da a Play antes de cada sesión |
| `plugins/configs/ur_config.yaml` | Configuración (IP de cada robot, posición inicial y tolerancia, timeout del leader) |
| `plugins/ursim/` | Dos simuladores URSim en Docker para probar sin hardware |
| `plugins/tests/` | Tests de los plugins |
| `src/lerobot/common/train_utils.py` | Arreglo para que `lerobot-train` no falle en Windows al crear `checkpoints/last` |

Diferencias exactas con el original: `git diff main...lerobot/ur-plugin --stat -- src plugins tests`.

## Instalación

Requisitos: Miniconda/Miniforge y Python 3.12.

```bash
conda create -y -n lerobot python=3.12
conda activate lerobot
conda install "ffmpeg=8.*" -c conda-forge

git clone https://github.com/blv-o/lerobot.git
cd lerobot
git switch lerobot/ur-plugin
python -m pip install -e ".[core_scripts,training,feetech]"
python -m pip install -e plugins/packages/ur_core -e plugins/packages/lerobot_robot_ur_follower -e plugins/packages/lerobot_teleoperator_ur_leader
```

Comprobación: `python -c "import lerobot; print(lerobot.__file__)"` debe apuntar a esta carpeta, no a `site-packages`.

## Uso

Guía de teleoperación (preparar los robots, lanzar, parar): [`plugins/README.md`](plugins/README.md).

## Dependencias

- `pyproject.toml` (raíz): el de LeRobot.
- `plugins/packages/*/pyproject.toml`: las dependencias propias (p. ej. el cliente RTDE de UR y `pyyaml`, en `ur_core`).

## Licencia

Apache 2.0, igual que el original (ver `LICENSE`). Basado en LeRobot © The HuggingFace Inc. team.
