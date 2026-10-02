# LeRobot para robots UR

Fork de [huggingface/lerobot](https://github.com/huggingface/lerobot) adaptado a brazos Universal Robots (UR3e), desarrollado en el TFM del Máster en Industria 4.0 (UPC).

Instala la librería `lerobot` completa (`src/`, sin cambios salvo lo indicado abajo) más unos plugins que añaden un robot follower y un teleoperador leader UR controlados por RTDE. Toda la documentación, ejemplos, Dockerfiles y CI del original se han retirado de este repositorio: para eso, consulta [el repositorio original](https://github.com/huggingface/lerobot) y [su documentación](https://huggingface.co/docs/lerobot).

## Qué añade respecto al original

| Dónde | Qué |
|---|---|
| `plugins/lerobot_robot_ur_follower/` | Robot `ur_follower`: UR que sigue las consignas por RTDE (`servoj` en un URScript que lee registros RTDE) |
| `plugins/lerobot_teleoperator_ur_leader/` | Teleoperador `ur_leader`: UR en freedrive (se mueve a mano) del que solo se lee la posición articular |
| `plugins/ur_teleop_core/` | Lógica común a ambos, con el cliente RTDE oficial de UR |
| `plugins/ursim/` | Dos URSim 5.25.2 (fijados por digest) en Docker para probar sin hardware |
| `plugins/tests/` | Suite de los plugins |
| `src/lerobot/common/train_utils.py` | `lerobot-train` no se cae en Windows al crear `checkpoints/last` (usa un *junction* si no se puede crear el symlink) |

Para ver exactamente qué difiere del original en código: `git diff main...lerobot/ur-plugin --stat -- src plugins tests` (`main` es una copia sin cambios del upstream).

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
python -m pip install -e plugins/ur_teleop_core -e plugins/lerobot_robot_ur_follower -e plugins/lerobot_teleoperator_ur_leader
```

Comprobación: `python -c "import lerobot; print(lerobot.__file__)"` debe apuntar a esta carpeta (`.../lerobot/src/lerobot/`), no a `site-packages`.

## Tests

```bash
pytest plugins/tests              # sin URSim ni hardware
pytest plugins/tests -m ursim     # con los URSim de plugins/ursim levantados (ver su README)
pytest tests                      # suite original de LeRobot
```

## Ramas

- `main`: copia del upstream, solo como referencia para comparar y traer actualizaciones.
- `fix/windows-last-checkpoint`: arreglo de Windows + retirada de lo que no aplica a este proyecto. Base de las demás ramas.
- `lerobot/ur-plugin` (rama por defecto): fork completo con los plugins UR; aquí llega el trabajo de cada spec.

## Licencia

Apache 2.0, igual que el original (ver `LICENSE`). Basado en LeRobot © The HuggingFace Inc. team.
