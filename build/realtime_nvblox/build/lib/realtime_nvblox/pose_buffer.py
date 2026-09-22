from __future__ import annotations

from collections import deque
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from realtime_nvblox.types import PoseSample


class PoseBuffer:
    def __init__(self, maxlen: int = 512):
        self._samples: deque[PoseSample] = deque(maxlen=maxlen)
        self._cv = threading.Condition()

    def add(self, sample: PoseSample) -> bool:
        with self._cv:
            if self._samples and sample.timestamp_ns < self._samples[-1].timestamp_ns:
                return False
            if self._samples and sample.timestamp_ns == self._samples[-1].timestamp_ns:
                self._samples.pop()
            self._samples.append(sample)
            self._cv.notify_all()
            return True

    def latest(self) -> PoseSample | None:
        with self._cv:
            return self._samples[-1] if self._samples else None

    def query(self, timestamp_ns: int, max_error_ms: float = 50.0, wait_ms: float = 25.0,
              allow_nearest: bool = True) -> PoseSample | None:
        deadline = time.monotonic() + max(0.0, wait_ms) / 1000.0
        with self._cv:
            while True:
                result = self._query_locked(timestamp_ns, max_error_ms)
                if result is not None:
                    return result
                if time.monotonic() >= deadline:
                    return self._nearest_locked(timestamp_ns, max_error_ms) if allow_nearest else None
                self._cv.wait(timeout=max(0.0, deadline - time.monotonic()))

    def _nearest_locked(self, ts: int, max_error_ms: float) -> PoseSample | None:
        if not self._samples:
            return None
        nearest = min(self._samples, key=lambda x: abs(x.timestamp_ns - ts))
        if abs(nearest.timestamp_ns - ts) > max_error_ms * 1e6:
            return None
        return PoseSample(ts, nearest.T_world_rig.copy(), nearest.tracking_ok, nearest.slam_T_world_rig,
                          pose_error_ms=abs(nearest.timestamp_ns - ts) / 1e6)

    def _query_locked(self, ts: int, max_error_ms: float) -> PoseSample | None:
        if not self._samples:
            return None
        samples = list(self._samples)
        before = None
        after = None
        for sample in reversed(samples):
            if sample.timestamp_ns <= ts:
                before = sample
                break
        for sample in samples:
            if sample.timestamp_ns >= ts:
                after = sample
                break
        if before is None or after is None:
            return None
        max_error_ns = max_error_ms * 1e6
        if ts - before.timestamp_ns > max_error_ns or after.timestamp_ns - ts > max_error_ns:
            return None
        if before.timestamp_ns == after.timestamp_ns:
            return PoseSample(ts, before.T_world_rig.copy(), before.tracking_ok)

        alpha = (ts - before.timestamp_ns) / (after.timestamp_ns - before.timestamp_ns)
        p0 = before.T_world_rig[:3, 3]
        p1 = after.T_world_rig[:3, 3]
        p = (1.0 - alpha) * p0 + alpha * p1
        rots = Rotation.from_matrix(np.stack([before.T_world_rig[:3, :3], after.T_world_rig[:3, :3]]))
        rot = Slerp([0.0, 1.0], rots)([alpha]).as_matrix()[0]
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = rot.astype(np.float32)
        T[:3, 3] = p.astype(np.float32)
        return PoseSample(ts, T, before.tracking_ok and after.tracking_ok,
                          pose_error_ms=max(ts - before.timestamp_ns, after.timestamp_ns - ts) / 1e6,
                          interpolated=True)
