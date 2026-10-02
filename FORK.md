# Fork `blv-o/lerobot` (TFM LeRobot + UR)

Fork de [huggingface/lerobot](https://github.com/huggingface/lerobot) para el TFM del Máster en Industria 4.0 (UPC). **Casi todo el repositorio es upstream sin cambios**; no se borra nada del original para que las actualizaciones desde Hugging Face no den conflictos y para que la diferencia con el original sea exactamente lo propio.

## Qué es propio

| Rama | Qué cambia | Archivos |
|---|---|---|
| `fix/windows-last-checkpoint` | `lerobot-train` no se cae en Windows al crear `checkpoints/last` (usa un *junction* si no se puede crear el symlink) | `src/lerobot/common/train_utils.py`, `tests/utils/test_train_utils.py` |
| `spec/003-ur-teleop` (sale de la anterior) | Teleoperación leader/follower de dos UR3e por RTDE: plugins `ur_follower`/`ur_leader`, librería `ur_teleop_core`, URSim en Docker y su suite de tests | `plugins/**` (solo archivos nuevos) y este `FORK.md` |

`main` sigue al upstream y no lleva nada propio.

## Ver exactamente lo que difiere del original

```bash
git diff main...spec/003-ur-teleop --stat
```

O en GitHub: <https://github.com/huggingface/lerobot/compare/main...blv-o:lerobot:spec/003-ur-teleop>

## Instalar

```bash
conda activate lerobot
git switch spec/003-ur-teleop
pip install -e ".[core_scripts,training,feetech]"
pytest plugins/tests          # suite de los plugins, sin URSim
```

Comprobar que se usa esta copia: `python -c "import lerobot; print(lerobot.__file__)"` no debe contener `site-packages`.

## Ver solo lo esencial en local

`docs/`, `examples/`, `media/`, `docker/`, `utils/`, `scripts/` y `.github/` son del upstream y no se usan en el TFM. Para ocultarlos del disco sin borrarlos de git:

```bash
git sparse-checkout set src plugins tests   # los archivos de la raíz se mantienen
git sparse-checkout disable                 # volver a ver todo
```
