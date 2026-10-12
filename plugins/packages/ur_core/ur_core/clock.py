"""Reloj único de ur_core para medir intervalos y marcar eventos.

No es `time.monotonic_ns()` porque en Python 3.12 sobre Windows usa GetTickCount64 y avanza a
saltos de 15,6 ms, más que un paquete RTDE del follower (8 ms a 125 Hz). `time.perf_counter_ns()`
también es monótono y resuelve a 100 ns (QueryPerformanceCounter en Windows, CLOCK_MONOTONIC en
Linux). Nunca `time.time()`: el reloj de pared puede saltar.
"""

import time


def now_ns() -> int:
    return time.perf_counter_ns()
