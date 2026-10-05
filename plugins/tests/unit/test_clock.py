"""El reloj de ur_core resuelve muy por debajo de un ciclo de servo (8 ms a 125 Hz, 2 ms a 500 Hz).

En Python 3.12 sobre Windows, `time.monotonic_ns()` usa GetTickCount64 y avanza a saltos de
15,6 ms: la interpolación iría a escalones y el periodo medido del bucle no significaría nada.
"""

import multiprocessing

from ur_core.clock import now_ns


def test_now_ns_resolves_well_below_a_millisecond() -> None:
    values = set()
    t0_ns = now_ns()
    while now_ns() - t0_ns < 20_000_000:  # 20 ms
        values.add(now_ns())
    ordered = sorted(values)
    smallest_step_ns = min(b - a for a, b in zip(ordered, ordered[1:], strict=False))
    assert smallest_step_ns < 100_000  # < 0,1 ms


def test_now_ns_never_goes_backwards() -> None:
    previous = now_ns()
    for _ in range(100_000):
        current = now_ns()
        assert current >= previous
        previous = current


def _child_now(queue: "multiprocessing.Queue[int]") -> None:
    queue.put(now_ns())


def test_now_ns_is_comparable_between_parent_and_spawned_child() -> None:
    """El padre marca las consignas y el hijo calcula su edad: deben compartir el mismo reloj."""
    ctx = multiprocessing.get_context("spawn")
    queue: multiprocessing.Queue[int] = ctx.Queue()
    before_ns = now_ns()
    child = ctx.Process(target=_child_now, args=(queue,))
    child.start()
    child_ns = queue.get(timeout=30)
    child.join(timeout=30)
    after_ns = now_ns()
    assert before_ns <= child_ns <= after_ns
