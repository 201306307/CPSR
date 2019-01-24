from typing import List, Tuple


class Navigation:
    """Class for short-term path planning."""

    def __init__(self):
        """Navigation class initializer."""
        # TODO: Add code if necessary.
        pass

    def explore(self, measurements: List[float]) -> Tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            measurements: Distance from every sensor to the closest obstacle [m].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: Compute v and w with your algorithm
        v = 0.2
        w = 0.5

        return v, w
