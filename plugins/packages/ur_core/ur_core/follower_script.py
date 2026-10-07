"""URScript del follower a partir de la configuración.

El programa vive en `follower_control.script` (legible y editable como URScript); aquí solo se
sustituyen sus parámetros, para que lo que corre en el robot salga siempre del mismo YAML.
"""

from pathlib import Path

from ur_core.config import ServoConfig, WatchdogConfig

TEMPLATE_PATH = Path(__file__).with_name("follower_control.script")


def render_follower_script(servo: ServoConfig, watchdog: WatchdogConfig) -> str:
    text = TEMPLATE_PATH.read_text(encoding="utf-8")
    values = {
        # Frecuencia mínima de escritura del registro enable. Medido en URSim, el controlador no
        # para al primer periodo sin dato sino tras ~2: con 2 / stop_s el robot para en ~stop_s.
        "__WATCHDOG_HZ__": 2 / watchdog.stop_s,
        "__SERVO_T_S__": 1 / servo.hz,
        "__LOOKAHEAD_S__": servo.lookahead_s,
        "__GAIN__": servo.gain,
    }
    for token, value in values.items():
        text = text.replace(token, repr(value))
    return text
