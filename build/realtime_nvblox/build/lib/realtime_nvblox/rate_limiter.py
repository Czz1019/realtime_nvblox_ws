from __future__ import annotations

import time


class RateLimiter:
    def __init__(self, hz: float):
        self.period_s = 0.0 if hz <= 0 else 1.0 / float(hz)
        self.last_s = -1e30

    def ready(self, now_s: float | None = None) -> bool:
        if self.period_s <= 0.0:
            return True
        now_s = time.monotonic() if now_s is None else now_s
        if now_s - self.last_s >= self.period_s:
            self.last_s = now_s
            return True
        return False
