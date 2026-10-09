# Dos URSim para la teleoperación UR

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

El código nunca enciende, suelta frenos ni cambia de modo (lo que se opera en el robot se opera
desde el TP).

1. Abrir `http://127.0.0.2:6080/vnc.html` (follower) o `http://127.0.0.3:6080/vnc.html` (leader).
2. Encender el robot: botón rojo de abajo a la izquierda → **ON** → **START** (suelta frenos) → **Exit**.
3. **Solo en el follower**, el programa del TP:
   - `docker-compose.yml` monta `plugins/tp/follower_control.script` en la carpeta de programas (no hay
     que copiarlo).
   - Programa `follower_tp.urp` con un nodo Script que cargue ese fichero. Darle a Play antes de cada
     sesión y tras cada parada.
   - El script no puede llamarse como el `.urp`: al guardar `X.urp`, PolyScope genera `X.script` y lo pisa.
4. **Solo para los tests `ursim`**, activar el control remoto (los tests cargan el programa y le dan a
   Play por el Dashboard):
   - Menú ☰ (arriba a la derecha) → **Settings** → **System** → **Remote Control** → **Enable** → **Exit**.
   - Con el selector de modo de la barra superior (icono del TP), pasar de **Local** a **Remote**.
   - No sobrevive a un `docker compose stop`/`up`: hay que repetirlo.
5. Llevar ambos robots a la posición inicial del YAML (`start_pose_deg`) desde la pestaña **Move**.
