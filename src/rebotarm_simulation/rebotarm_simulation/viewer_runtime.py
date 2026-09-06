from __future__ import annotations

import time
from typing import Callable


# Native MuJoCo objects must outlive a passive viewer that failed to release its
# model. Retaining the full ownership graph is safer than risking use-after-free.
RETAINED_UNSAFE_VIEWERS: list[tuple[object, object, object, object]] = []


def close_viewer_then_sim(
    viewer,
    simulation,
    model,
    data,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    timeout: float = 5.0,
) -> None:
    """Close a passive viewer before releasing its native simulation state."""
    if viewer is None:
        simulation.close()
        return
    try:
        viewer.close()
    except BaseException:
        model_is_released = False
        try:
            model_is_released = viewer.m is None
        except BaseException:
            # Releasing native state would be unsafe when its lifecycle signal
            # is unavailable.
            pass
        if model_is_released:
            simulation.close()
        else:
            RETAINED_UNSAFE_VIEWERS.append((viewer, simulation, model, data))
        raise

    viewer_model = getattr(viewer, "m", None)
    if viewer_model is None:
        simulation.close()
        return
    started = clock()
    while viewer_model is not None:
        if clock() - started >= timeout:
            RETAINED_UNSAFE_VIEWERS.append((viewer, simulation, model, data))
            raise TimeoutError("MuJoCo passive viewer did not finish closing")
        sleep(0.01)
        viewer_model = getattr(viewer, "m", None)
    simulation.close()
