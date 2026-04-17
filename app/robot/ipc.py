"""Small IPC primitives used to mimic ROS topic semantics with threads.

`LatestQueue` keeps only the newest message so slow consumers do not build up
latency. That is a better fit for real-time robot control than an unbounded
FIFO queue.
"""

import queue
from typing import Generic, Optional, TypeVar

T = TypeVar("T")


class LatestQueue(Generic[T]):
    def __init__(self, maxsize: int = 1) -> None:
        self._queue: "queue.Queue[T]" = queue.Queue(maxsize=max(1, maxsize))

    def put_latest(self, item: T) -> None:
        while True:
            try:
                self._queue.put_nowait(item)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    return

    def get(self, timeout: Optional[float] = None) -> T:
        return self._queue.get(timeout=timeout)
