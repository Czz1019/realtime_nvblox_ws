from __future__ import annotations

import threading
from typing import Generic, Optional, Tuple, TypeVar

T = TypeVar('T')


class LatestSlot(Generic[T]):
    """Thread-safe latest-only handoff.

    The producer never blocks on a growing queue.  Each put replaces the
    previous value, which is what a low-latency map/ESDF pipeline wants: stale
    outputs are skipped instead of accumulating latency.
    """

    def __init__(self) -> None:
        self._cv = threading.Condition()
        self._value: Optional[T] = None
        self._sequence = 0
        self._closed = False

    def put(self, value: T) -> int:
        with self._cv:
            if self._closed:
                return self._sequence
            self._value = value
            self._sequence += 1
            self._cv.notify_all()
            return self._sequence

    def get_if_newer(self, last_sequence: int) -> Tuple[int, Optional[T]]:
        with self._cv:
            if self._sequence <= last_sequence:
                return last_sequence, None
            return self._sequence, self._value

    def wait_newer(self, last_sequence: int, timeout: float | None = None) -> Tuple[int, Optional[T]]:
        with self._cv:
            self._cv.wait_for(
                lambda: self._closed or self._sequence > last_sequence,
                timeout=timeout,
            )
            if self._closed or self._sequence <= last_sequence:
                return last_sequence, None
            return self._sequence, self._value

    def latest(self) -> Tuple[int, Optional[T]]:
        with self._cv:
            return self._sequence, self._value

    def close(self) -> None:
        with self._cv:
            self._closed = True
            self._cv.notify_all()
