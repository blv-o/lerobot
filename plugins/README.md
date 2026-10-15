# Guía de uso: teleoperación UR → UR

Un UR leader (se mueve a mano) y un UR follower (lo sigue), con `lerobot-teleoperate` o `lerobot-record`.
Aquí con los dos URSim de `plugins/ursim/`; con robots reales cambian las IP y el perfil del YAML.

## 1. Arrancar los URSim

Follower en `127.0.0.2` y leader en `127.0.0.3` (URSim 5.25.2), con noVNC en el 6080 de cada IP.

```
docker compose -f plugins/ursim/docker-compose.yml up -d     # arrancar (≈1 min hasta que PolyScope responde)
pytest plugins/tests/ursim/test_ursim_ports.py -m ursim      # comprobar que los dos responden por RTDE
docker compose -f plugins/ursim/docker-compose.yml stop      # parar sin borrar el estado
```

Con Docker Desktop los puertos aceptan conexiones antes de que URSim arranque: por eso la comprobación
pide la versión del controlador por RTDE.

URSim del follower ejecuta el `plugins/tp/follower_control.script` del checkout principal (el compose lo monta de ahí, ver `plugins/ursim/docker-compose.yml`): es el script de la rama activa en ese checkout. El `load` del Dashboard (tests) relee el archivo. El `.urp` guarda además una copia; sin comprobar si el Play del TP la usa (ver sección 2).

## 2. Preparar los robots (a mano, en PolyScope)

El código nunca enciende, suelta frenos ni cambia de modo. Abrir el PolyScope de cada uno:
follower `http://127.0.0.2:6080/vnc.html`, leader `http://127.0.0.3:6080/vnc.html`.

1. Los dos: botón rojo abajo a la izquierda → **ON** → **START** → **Exit**.
2. Los dos: pestaña **Move** → posición inicial del YAML (`[0, -90, 90, -90, -90, 0]` ± 2°).
3. Solo el follower: abrir `follower_tp.urp`.
   - La primera vez se crea: programa nuevo con un nodo Script que carga `follower_control.script`
     (el compose ya lo monta en la carpeta de programas desde `plugins/tp/`).
   - No llamarlo `follower_control.urp`: al guardar `X.urp`, PolyScope genera `X.script` y pisa el script.
   - Tras cambiar `follower_control.script`, volver a guardar el `.urp` en el TP: el `.urp` guarda una copia del script.
   - Robot real: copiar `follower_control.script` al robot con un USB (o por red a la carpeta de programas) y crear el `.urp` igual.

Modo del follower (selector de la barra superior):
- **Local** para teleoperar.
- **Remote** solo para los tests `ursim` (en Remote, el Play del TP no funciona, dan Play por el Dashboard).
  - La primera vez: menú ☰ → **Settings** → **System** → **Remote Control** → **Enable**.
  - Se pierde con `docker compose stop`/`up` o al recrear el contenedor: volver a ponerlo.

Red (robots reales):
- Red dedicada para el PC y los robots.
- IP fija del PC en la misma subred que los robots.
- En PolyScope del follower y del leader: restringir el acceso entrante a esa subred y desactivar los servicios que no se usen (dejar RTDE, puerto 30004).

## 3. Copiar el YAML

```powershell
Copy-Item lerobot\plugins\configs\ur_config.yaml project\outputs\ur_config.yaml
```

- Con los URSim no hay que cambiar nada (`profile: sim`).
- Robots reales: `profile` (`real`, o `{leader: real, follower: sim}`) y sus IP `real:`.
- Velocidad máxima e interpolación se ajustan solo en el programa del TP (`plugins/tp/follower_control.script`).

## 4. Lanzar

1. **Play** en el TP del follower, antes de cada sesión y tras cada parada.
2. Activar el entorno y moverse al repo:
   ```
   conda activate lerobot
   cd lerobot
   ```
3. Lanzar, guardando el log de la sesión:
   ```
    lerobot-teleoperate --robot.type=ur_follower --robot.config_path="..\project\outputs\ur_config.yaml"  
    --teleop.type=ur_leader --teleop.config_path="..\project\outputs\ur_config.yaml" --fps=30
   ```
   Añadir `2>&1 | Tee-Object ur_session.log` para guardar el log
   En Linux: `... 2>&1 | tee ur_session.log`.
4. Para grabar, `lerobot-record` con los mismos `--robot.*` y `--teleop.*`, `--dataset.fps=30` y `--play_sounds=false` (la voz bloquea al pulsar Ctrl+C y retrasa la parada).

- `--fps=30`: casa con `TARGET_PERIOD_S` (33 ms) del programa del TP, que llega a cada consigna en 5 vueltas de 8 ms (40 ms). Sin él, `lerobot-teleoperate` usa 60. En `lerobot-record` es `--dataset.fps` (por defecto 30).
- Pregunta `yes/no` dos veces (primero el leader, luego el follower). Con `yes` comprueba la posición; si algo falla, lo dice y vuelve a preguntar.
- Con `no`: «arranque cancelado por el operador» y sale sin mover nada.
- La pregunta del leader pide un programa de freedrive en Play: solo con un UR real. En URSim, contestar `yes` y mover el leader desde **Move**.
- Al grabar, las teclas de LeRobot (Esc, flechas) se captan en todo el escritorio: no pulsarlas en la pestaña noVNC de URSim ni en otra ventana.

## 5. Parar

- Parar siempre con **Ctrl+C**: parada limpia. El programa del TP frena con `stopj` y termina.
  - En `lerobot-record` no es instantáneo: antes LeRobot termina de guardar (`finalize()`) y el robot sigue en Play, quieto, unos segundos.
  - No pulsar Ctrl+C dos veces. Si se cuelga ahí, **Stop** en el TP.
  - Si aún no había llegado ninguna consigna (p. ej. «primera consigna lejos»), el programa sigue en Play con el robot quieto.
- No dar Stop y Play en el TP con LeRobot en marcha (p. ej. mientras guarda un episodio): LeRobot para con «el programa del follower no está en ejecución» y el robot no se mueve.
- Al grabar, dejar el leader quieto mientras se guarda el episodio: el programa no limita la aceleración y el robot arrancaría de golpe (a 60 °/s) hacia la nueva pose.
- Si el PC deja de escribir sin esa parada (PC colgado, red caída, **Ctrl+Break**, proceso matado, PC apagado):
  - El robot se queda quieto en la última consigna, con el programa en Play. Cortar la conexión RTDE no lo para.
  - Se para con **Stop** en el TP.

- Antes de relanzar, en todos los casos: **Stop** y **Play** en el TP.
- Al acabar cada sesión: **Stop** al programa del follower en el TP. Un programa armado en Play acepta consignas de cualquier cliente RTDE de la red.

## Problemas frecuentes

- «el programa del follower no está en marcha: dale a Play en el TP» → **Play** en el follower.
- «el programa del follower no está en ejecución» al empezar: el programa seguía de la sesión anterior → **Stop** y **Play** en el TP.
- «registros de entrada … no disponibles» → los usa otro cliente RTDE o un bus de campo: cerrar el proceso anterior que siga vivo (p. ej. tras un PC colgado) o desactivar el bus de campo en PolyScope.
- «primera consigna lejos» → leader y follower no estaban juntos en la posición inicial: llevarlos allí, **Stop** y **Play**, y relanzar.
- «se perdió la conexión» (RTDE) → revisar cable y red; **Stop** y **Play** en el TP.
- `safety_mode=…` o `robot_mode=…` → mirar el TP, desbloquear la parada, **Stop** y **Play**.
- Error del leader (también al conectar) → programa de freedrive en Play y cable del leader.
- «no se puede leer la posición» → comprobar la IP del YAML y que el robot esté encendido.
- «fuera de tolerancia» → llevar ese robot a la posición inicial desde **Move** y contestar `yes`.
- Tests `ursim` con «Failed to execute: play» → el follower está apagado o en Local (sección 2).
- `lerobot-calibrate`: no hay nada que calibrar; contestar `no`.
- Cerrar la terminal depende de la terminal:
  - Warp manda una interrupción, como Ctrl+C: parada limpia.
  - La ventana clásica de PowerShell aborta el proceso sin parada: el robot queda quieto en Play.
