# Guía de uso: teleoperación UR → UR

Un UR leader (se mueve a mano) y un UR follower (lo sigue), con `lerobot-teleoperate` o `lerobot-record`.
Aquí con los dos URSim de `plugins/ursim/`; con robots reales cambian las IP y el perfil del YAML.

## 1. Arrancar los URSim

Follower en `127.0.0.2` y leader en `127.0.0.3` (URSim 5.25.2), con noVNC en el 6080 de cada IP.

```
docker compose -f plugins/ursim/docker-compose.yml up -d     # arrancar (≈1 min hasta que PolyScope responde)
pytest plugins/tests -m ursim                                # comprobar que los dos responden
docker compose -f plugins/ursim/docker-compose.yml stop      # parar sin borrar el estado
```

## 2. Preparar los robots (a mano, en PolyScope)

El código nunca enciende, suelta frenos ni cambia de modo. Abrir el PolyScope de cada uno:
follower `http://127.0.0.2:6080/vnc.html`, leader `http://127.0.0.3:6080/vnc.html`.

1. Los dos: botón rojo abajo a la izquierda → **ON** → **START** → **Exit**.
2. Los dos: pestaña **Move** → posición inicial del YAML (`[0, -90, 90, -90, -90, 0]` ± 2°).
3. Solo el follower: abrir `follower_tp.urp`.
   - La primera vez se crea: programa nuevo con un nodo Script que carga `follower_control.script`
     (el compose ya lo monta en la carpeta de programas desde `plugins/tp/`).
   - No llamarlo `follower_control.urp`: al guardar `X.urp`, PolyScope genera `X.script` y pisa el script.

Modo del follower (selector de la barra superior):
- **Local** para teleoperar: en Remote, el Play del TP no funciona.
- **Remote** solo para los tests `ursim` (dan Play por el Dashboard).
  - La primera vez: menú ☰ → **Settings** → **System** → **Remote Control** → **Enable**.
  - Se pierde con `docker compose stop`/`up` o al recrear el contenedor: volver a ponerlo.

## 3. Copiar el YAML

```powershell
Copy-Item plugins\configs\ur_config.yaml mi_ur_config.yaml
```

- Con los URSim no hay que cambiar nada (`profile: sim`).
- Robots reales: `profile` (`real`, o `{leader: real, follower: sim}`) y sus IP `real:`.
- `servo.hz` y `watchdog.stop_ms` deben coincidir con el programa del TP.

## 4. Lanzar

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

## 5. Parar

- **Ctrl+C**: parada limpia. El programa del TP termina: otra vez **Play** antes de relanzar.
- Cerrar la consola o **Ctrl+Break**: el robot para solo (~0,5 s) y queda en parada de protección. Desbloquearla en el TP y **Play**.

## Problemas frecuentes

- «el programa del follower no está en marcha: dale a Play en el TP» → **Play** en el follower.
- «fuera de tolerancia» → llevar ese robot a la posición inicial desde **Move** y contestar `yes`.
- Tests `ursim` con «Failed to execute: play» → el follower está apagado o en Local (sección 2).
- `lerobot-calibrate`: no hay nada que calibrar; contestar `no`.
