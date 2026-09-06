from __future__ import annotations

import math
import time
from typing import Callable


class FreshnessTracker:
    def __init__(self, monotonic: Callable[[], float] = time.monotonic) -> None:
        self._monotonic = monotonic
        self._received_at: dict[str, float] = {}

    def touch(self, key: str) -> None:
        self._received_at[str(key)] = float(self._monotonic())

    def invalidate(self, key: str) -> None:
        self._received_at.pop(str(key), None)

    def is_fresh(self, key: str, max_age_sec: float) -> bool:
        maximum = float(max_age_sec)
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise ValueError("max_age_sec must be finite and positive")
        received = self._received_at.get(str(key))
        if received is None:
            return False
        age = float(self._monotonic()) - received
        return 0.0 <= age <= maximum
