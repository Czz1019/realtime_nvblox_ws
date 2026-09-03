from __future__ import annotations

import queue
from threading import Lock
from typing import Generic, Optional, TypeVar

T = TypeVar('T')


class DropOldestQueue(Generic[T]):
    """Bounded queue that always preserves the newest data."""

    def __init__(self, maxsize: int):
        self._q: queue.Queue[T] = queue.Queue(maxsize=max(1, int(maxsize)))
        self._lock = Lock()
        self.dropped = 0

    def put(self, item: T) -> None:
        with self._lock:
            if self._q.full():
                try:
                    self._q.get_nowait()
                    self.dropped += 1
                except queue.Empty:
                    pass
            self._q.put_nowait(item)

    def get(self, timeout: Optional[float] = None) -> T:
        return self._q.get(timeout=timeout)

    def get_nowait(self) -> T:
        return self._q.get_nowait()

    def qsize(self) -> int:
        return self._q.qsize()

    def empty(self) -> bool:
        return self._q.empty()
