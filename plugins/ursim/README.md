# Dos URSim para SPEC_003

Follower en `127.0.0.2` y leader en `127.0.0.3`, con los puertos estándar de UR (RTDE 30004, secundaria
30002, dashboard 29999) y noVNC en el 6080 de cada IP. URSim 5.25.2, fijado por digest.

```
docker compose -f plugins/ursim/docker-compose.yml up -d     # arrancar (≈1 min hasta que PolyScope responde)
pytest plugins/tests -m ursim                                # comprobar que los dos responden por RTDE
docker compose -f plugins/ursim/docker-compose.yml stop      # parar sin borrar el estado
```

Con Docker Desktop, los puertos aceptan conexiones aunque URSim aún no haya arrancado: el test `ursim`
pide la versión del controlador por RTDE para saber que hay un robot detrás.

## Preparar los robots (a mano, desde PolyScope por noVNC)

El código nunca enciende, suelta frenos ni cambia de modo (principio de SPEC_003: lo que se opera en el
robot se opera desde el TP).

1. Abrir `http://127.0.0.2:6080/vnc.html` (follower) o `http://127.0.0.3:6080/vnc.html` (leader).
2. Encender el robot: botón rojo de abajo a la izquierda → **ON** → **START** (suelta frenos) → **Exit**.
3. **Solo en el follower**, activar el control remoto:
   - Menú ☰ (arriba a la derecha) → **Settings** → **System** → **Remote Control** → **Enable** → **Exit**.
   - Con el selector de modo de la barra superior (icono del TP), pasar de **Local** a **Remote**.
4. Llevar ambos robots a la posición inicial del YAML (`start_pose_deg`) desde la pestaña **Move**.

Pendiente de verificar (SPEC_003, dudas abiertas): si la activación de Remote Control sobrevive a un
`docker compose stop`/`up` con el volumen de `programs/`.
