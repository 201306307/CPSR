import math
import numpy as np
import os

from map import Map
from typing import List, Tuple

import matplotlib
matplotlib.use("TkAgg")  # Prevents matplotlib from crashing in macOS
from matplotlib import pyplot as plt


class Planning:
    """Class to plan the optimal path to a given location."""

    def __init__(self, map_object: Map, action_costs: Tuple[float, float, float, float]):
        """Planning class initializer.

        Args:
            map_object: Map of the environment.
            action_costs: Cost of of moving one cell left, right, up, and down.

        """
        self._map = map_object

        self._actions = np.array([
            (-1, 0),  # Move one cell left
            (0, 1),   # Move one cell right
            (1, 0),   # Move one cell up
            (0, -1)   # Move one cell down
        ])

        self._action_costs = action_costs

    def a_star(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Computes the optimal path to a given goal location using the A* algorithm.

        Args:
            start: Initial location in (x, y) format.
            goal: Destination in (x, y) format.

        Returns:
            Path to the destination. The first value corresponds to the initial location.

        """
        # TODO: Complete with your code.
        pass

    @staticmethod
    def smooth_path(path, data_weight: float = 0.1, smooth_weight: float = 0.1, tolerance: float = 1e-6) -> \
            List[Tuple[float, float]]:
        """Computes a smooth trajectory from a Manhattan-like path.

        Args:
            path: Non-smoothed path to the goal (start location first).
            data_weight: The larger, the more similar the output will be to the original path.
            smooth_weight: The larger, the smoother the output path will be.
            tolerance: The algorithm will stop when after an iteration the smoothed path changes less than this value.

        Returns: Smoothed path (initial location first) in (x, y) format.

        """
        # TODO: Complete with your code.
        pass

    @staticmethod
    def plot(axes, path: List[Tuple[float, float]], smoothed_path: List[Tuple[float, float]] = ()):
        """Draws a path.

        Args:
            axes: Figure axes.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).

        Returns:
            axes: Modified axes.

        """
        x_val = [x[0] for x in path]
        y_val = [x[1] for x in path]

        axes.plot(x_val, y_val)  # Plot the path
        axes.plot(x_val[1:-1], y_val[1:-1], 'bo', markersize=4)  # Draw blue circles in every intermediate cell

        if smoothed_path:
            x_val = [x[0] for x in smoothed_path]
            y_val = [x[1] for x in smoothed_path]

            axes.plot(x_val, y_val, 'y')  # Plot the path
            axes.plot(x_val[1:-1], y_val[1:-1], 'yo', markersize=4)  # Draw yellow circles in every intermediate cell

        axes.plot(x_val[0], y_val[0], 'rs', markersize=7)  # Draw a red square at the start location
        axes.plot(x_val[-1], y_val[-1], 'g*', markersize=12)  # Draw a green star at the goal location

        return axes

    def show(self, path, smoothed_path=(), figure_number: int = 1, title: str = 'Path', blocking: bool = False,
             figure_size: Tuple[float, float] = (7, 7), save_figure: bool = False, save_dir: str = 'img'):
        """Displays a given path on the map.

        Args:
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).
            figure_number: Any existing figure with the same value will be overwritten.
            title: Plot title.
            blocking: True to stop program execution until the figure window is closed.
            figure_size: Figure window dimensions.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.

        """
        figure = plt.figure(figure_number, figsize=figure_size)
        figure.clf()

        axes = figure.add_subplot(1, 1, 1)
        axes = self._map.plot(axes)
        axes = self.plot(axes, path, smoothed_path)

        axes.set_title(title)
        figure.tight_layout()  # Reduce white margins

        if not blocking:
            plt.ion()          # Activate interactive mode
            plt.show()
            plt.pause(0.0001)  # Wait for 1 ms or the figure wont be displayed
        else:
            plt.show()

        if save_figure:
            if not os.path.isdir(save_dir):
                os.mkdir(save_dir)

            file_name = str(title.lower() + '.png')
            file_path = os.path.join(save_dir, file_name)
            figure.savefig(file_path, box_inches='tight')

    def _compute_heuristic(self, goal: Tuple[float, float]) -> np.ndarray:
        """Creates an admissible heuristic.

        Args:
            goal: Destination location in (x,y) coordinates.

        Returns:
            Admissible heuristic.

        """
        # TODO: Complete with your code.
        pass

    def _reconstruct_path(self, start: Tuple[float, float], goal: Tuple[float, float], ancestors: np.ndarray) -> \
            List[Tuple[float, float]]:
        """Computes the trajectory from the start to the goal location given the ancestors of a search algorithm.

        Args:
            start: Initial location in (x, y) format.
            goal: Goal location in (x, y) format.
            ancestors: Matrix that contains for every cell, None or the (x, y) ancestor from which it was opened.

        Returns: Path to the goal (start location first) in (x, y) format.

        """
        # TODO: Complete with your code.
        pass

    def _xy_to_rc(self, xy: Tuple[float, float]) -> Tuple[float, float]:
        """Converts (x, y) coordinates of a metric map to (row, col) coordinates of a grid map.

        Args:
            xy: (x, y) [m].

        Returns:
            rc: (row, col) starting from (0, 0) at the top left corner.

        """
        map_rows, map_cols = np.shape(self._map.grid_map)

        x = round(xy[0])
        y = round(xy[1])

        row = int(map_rows - (y + math.ceil(map_rows / 2.0)))
        col = int(x + math.floor(map_cols / 2.0))

        return row, col

    def _rc_to_xy(self, rc: Tuple[float, float]) -> Tuple[float, float]:
        """Converts (row, col) coordinates of a grid map to (x, y) coordinates of a metric map.

        Args:
            rc: (row, col) starting from (0, 0) at the top left corner.

        Returns:
            xy: (x, y) [m].

        """
        map_rows, map_cols = np.shape(self._map.grid_map)
        row, col = rc

        x = col - math.floor(map_cols / 2.0)
        y = map_rows - (row + math.ceil(map_rows / 2.0))

        return x, y


def test():
    """Function used to test the Planning class independently."""
    m = Map('map_project.json')

    start = (-4.0, -4.0)
    goal = (4.0, 4.0)
    action_costs = (1.0, 1.0, 1.0, 1.0)

    planning = Planning(m, action_costs)
    path = planning.a_star(start, goal)
    smoothed_path = planning.smooth_path(path, data_weight=0.1, smooth_weight=0.1)
    planning.show(path, smoothed_path, blocking=True)


if __name__ == '__main__':
    test()
