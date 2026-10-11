# Guía de uso: teleoperación UR → UR

Un UR leader (se mueve a mano) y un UR follower (lo sigue), con `lerobot-teleoperate` o `lerobot-record`.
Aquí con los dos URSim de `plugins/ursim/`; con robots reales cambian las IP y el perfil del YAML.

## 1. Preparar los URSim

Arrancarlos: `plugins/ursim/README.md`. Después, en el PolyScope de cada uno por noVNC
(follower `http://127.0.0.2:6080/vnc.html`, leader `http://127.0.0.3:6080/vnc.html`):

1. Los dos: botón rojo abajo a la izquierda → **ON** → **START** → **Exit**.
2. Los dos: pestaña **Move** → posición inicial del YAML (`[0, -90, 90, -90, -90, 0]` ± 2°).
3. Solo el follower: abrir `follower_tp.urp`, un programa con un nodo Script que carga
   `plugins/tp/follower_control.script` (la primera vez se crea a mano: `plugins/ursim/README.md`).

Modo del follower (selector de la barra superior):
- **Local** para teleoperar: en Remote, el Play del TP no funciona.
- **Remote** solo para los tests `ursim`: dan Play por el Dashboard.

## 2. Copiar el YAML

```powershell
Copy-Item plugins\configs\ur_config.yaml mi_ur_config.yaml
```

- Con los URSim no hay que cambiar nada (`profile: sim`).
- Robots reales: `profile` (`real`, o `{leader: real, follower: sim}`) y sus IP `real:`.
- `servo.hz` y `watchdog.stop_ms` deben coincidir con el programa del TP.

## 3. Lanzar

1. **Play** en el TP del follower, antes de cada sesión y tras cada parada.
2. En PowerShell, para que las tildes salgan bien en consola y en el log (una vez por consola):
   ```powershell
   $env:PYTHONUTF8 = "1"; [Console]::OutputEncoding = [Text.Encoding]::UTF8
   ```
3. Lanzar, guardando el log de la sesión:
   ```powershell
   lerobot-teleoperate --robot.type=ur_follower --robot.config_path=mi_ur_config.yaml `
     --teleop.type=ur_leader --teleop.config_path=mi_ur_config.yaml --fps=30 2>&1 | Tee-Object ur_session.log
   ```
   En Linux: `... 2>&1 | tee ur_session.log`.

- `--fps` = 1000 / `target_period_ms` (33 ms → 30). Sin él, LeRobot usa 60.
- Pregunta `yes/no` dos veces (follower y leader). Con `yes` comprueba la posición; si algo falla, lo dice y vuelve a preguntar.
- Con `no`: «arranque cancelado por el operador» y sale sin mover nada.
- La pregunta del leader pide un programa de freedrive en Play: solo con un UR real. En URSim, contestar `yes` y mover el leader desde **Move**.

## 4. Parar

- **Ctrl+C**: parada limpia. El programa del TP termina: otra vez **Play** antes de relanzar.
- Cerrar la consola o **Ctrl+Break**: el robot para solo (~0,5 s) y queda en parada de protección. Desbloquearla en el TP y **Play**.

## Problemas frecuentes

- «el programa del follower no está en marcha: dale a Play en el TP» → **Play** en el follower.
- «fuera de tolerancia» → llevar ese robot a la posición inicial desde **Move** y contestar `yes`.
- `lerobot-calibrate`: no hay nada que calibrar; contestar `no`.
