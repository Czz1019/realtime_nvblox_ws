from __future__ import annotations

from collections import defaultdict, deque
import json
from pathlib import Path
import threading
import time

import numpy as np


class RuntimeStats:
    def __init__(self):
        self.lock = threading.Lock()
        self.counts = defaultdict(int)
        self.last_counts = defaultdict(int)
        self.rates = defaultdict(float)
        self.latencies = defaultdict(lambda: deque(maxlen=512))
        self.last_rate_time = time.monotonic()

    def inc(self, name: str, n: int = 1) -> None:
        with self.lock:
            self.counts[name] += n

    def add_latency(self, name: str, ms: float) -> None:
        with self.lock:
            self.latencies[name].append(float(ms))

    def snapshot(self, extra: dict | None = None) -> dict:
        with self.lock:
            now = time.monotonic()
            dt = max(1e-6, now - self.last_rate_time)
            for key, value in self.counts.items():
                self.rates[key] = (value - self.last_counts[key]) / dt
                self.last_counts[key] = value
            self.last_rate_time = now
            latency = {}
            for key, values in self.latencies.items():
                if values:
                    a = np.asarray(values, dtype=np.float64)
                    latency[key] = {
                        'mean_ms': float(a.mean()),
                        'p50_ms': float(np.percentile(a, 50)),
                        'p95_ms': float(np.percentile(a, 95)),
                        'max_ms': float(a.max()),
                    }
            out = {
                'rates_hz': dict(self.rates),
                'totals': dict(self.counts),
                'latency': latency,
            }
            if extra:
                out.update(extra)
            return out

    def write_json(self, path: str | Path, extra: dict | None = None) -> Path:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.snapshot(extra), indent=2), encoding='utf-8')
        return path
