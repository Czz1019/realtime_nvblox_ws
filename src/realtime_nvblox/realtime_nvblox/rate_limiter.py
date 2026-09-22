from __future__ import annotations

import math
import time


class RateLimiter:
    """Preserve phase across jitter; discard accumulated debt after long stalls."""

    def __init__(self, hz: float):
        if not math.isfinite(hz):
            raise ValueError('Rate must be finite.')
        self.period_s = 0.0 if hz <= 0 else 1.0 / float(hz)
        self.next_s: float | None = None

    def remaining(self, now_s: float | None = None) -> float:
        if self.period_s <= 0.0 or self.next_s is None:
            return 0.0
        now_s = time.monotonic() if now_s is None else now_s
        return max(0.0, self.next_s - now_s)

    def ready(self, now_s: float | None = None) -> bool:
        if self.period_s <= 0.0:
            return True
        now_s = time.monotonic() if now_s is None else now_s
        if self.next_s is None:
            self.next_s = now_s + self.period_s
            return True
        if now_s + 1e-9 < self.next_s:
            return False
        if now_s - self.next_s >= 2.0 * self.period_s:
            self.next_s = now_s + self.period_s
        else:
            # Keep one missed deadline as credit. Rounding past it would discard
            # every early frame of an alternating early/late 30 Hz input.
            self.next_s += self.period_s
        return True
