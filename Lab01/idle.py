import time

from typing import Tuple


class Idle:
    """Class to ensure fixed scan loop sampling times."""

    def __init__(self, dt: float):
        """Idle class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt = dt
        self._start_time = time.time()

    def task(self) -> Tuple[float, bool]:
        """Delays execution until the sampling time is over.

        Returns:
            loop_time: Scan loop execution time [s].
            overflow: True if the scan loop exceeds the sampling time, false otherwise.

        """
        timestamp = time.time()

        loop_time = timestamp - self._start_time
        overflow = False if loop_time < self._dt else True

        while timestamp - self._start_time < self._dt:
            timestamp = time.time()

        self._start_time = timestamp

        return loop_time, overflow
