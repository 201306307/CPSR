import math
import numpy as np
import os

from map import Map
from typing import List, Tuple

import time

from numpy import dot, arccos, asanyarray

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

    def a_star(self, start: Tuple[float, float], goal: Tuple[float, float]):
        # -> List[Tuple[float, float]], np.ndarray:
        """Computes the optimal path to a given goal location using the A* algorithm.

        Args:
            start: Initial location in (x, y) format.
            goal: Destination in (x, y) format.

        Returns:
            Path to the destination. The first value corresponds to the initial location.

        """

        heuristic = self._compute_heuristic(goal)

        g = 0
        goal_rc = self._xy_to_rc(goal)
        start_rc = self._xy_to_rc(start)

        f = 0

        i = 1

        point = (start_rc[0], start_rc[1], heuristic[start_rc[0], start_rc[1]], g)
        
        open_list = []
        closed_list = []

        neighbours = []

        map_rows, map_cols = np.shape(self._map.grid_map)

        matrix_appended = np.full(shape = (map_rows, map_cols), fill_value = None)

        reconstruct_path = []

        matrix_appended[start_rc[0], start_rc[1]] = (start_rc[0], start_rc[1], g)


        while (point[0], point[1]) != goal_rc:
            d_x = point[0] - matrix_appended[point[0], point[1]][0]
            d_y = point[1] - matrix_appended[point[0], point[1]][1]
            direction = [d_x,d_y]

            new_direction = [-1,0] #RIGHT
            a = direction[0] * new_direction[0] + direction[1] * new_direction[1]
            c = np.dot(np.asanyarray(direction), np.asanyarray(new_direction)) # -> cosine of the angle

            if c == 1:
                neighbours.append((point[0] - 1, point[1], self._action_costs[0])) #STRAIGHT
            elif c == -1:
                neighbours.append((point[0] - 1, point[1], self._action_costs[1]))  # BACKWARDS
            elif c == 0:
                if d_x == 0 and d_y != 0:
                    #Se movía en vertical
                    if d_y > 0:
                        #Se movía aumentando y
                        if new_direction[0] > 0:
                            #Giro a izquierda
                            neighbours.append((point[0] - 1, point[1], self._action_costs[2]))
                        if new_direction[0] < 0:
                            #Giro a derecha
                            neighbours.append((point[0] - 1, point[1], self._action_costs[3]))
                    if d_y > 0:
                        #Se movía disminuyendo y
                        if new_direction[0] > 0:
                            #Giro a derecha
                            neighbours.append((point[0] - 1, point[1], self._action_costs[3]))
                        if new_direction[0] < 0:
                            #Giro a izquierda
                            neighbours.append((point[0] - 1, point[1], self._action_costs[2]))
                if d_y == 0 and d_x != 0:
                    #Se movía en horizontal
                    if d_x > 0:
                        #Se movía aumentando x
                        if new_direction[1] > 0:
                            #Giro a izquierda
                            neighbours.append((point[0] - 1, point[1], self._action_costs[2]))
                        if new_direction[1] < 0:
                            #Giro a derecha
                            neighbours.append((point[0] - 1, point[1], self._action_costs[3]))
                    if d_x < 0:
                        #Se movía aumentando x
                        if new_direction[1] > 0:
                            #Giro a derecha
                            neighbours.append((point[0] - 1, point[1], self._action_costs[3]))
                        if new_direction[1] < 0:
                            #Giro a izquierda
                            neighbours.append((point[0] - 1, point[1], self._action_costs[2]))
                else:
                    neighbours.append((point[0] - 1, point[1], 1)) #FIRST STEP

            new_direction = [1, 0] # LEFT
            a = direction[0] * new_direction[0] + direction[1] * new_direction[1]
            c = np.dot(np.asanyarray(direction), np.asanyarray(new_direction))  # -> cosine of the angle

            if c == 1:
                neighbours.append((point[0] + 1, point[1], self._action_costs[0]))  # STRAIGHT
            elif c == -1:
                neighbours.append((point[0] + 1, point[1], self._action_costs[1]))  # BACKWARDS
            elif c == 0:
                if d_x == 0 and d_y != 0:
                    #Se movía en vertical
                    if d_y > 0:
                        #Se movía aumentando y
                        if new_direction[0] > 0:
                            #Giro a izquierda
                            neighbours.append((point[0] + 1, point[1], self._action_costs[2]))
                        if new_direction[0] < 0:
                            #Giro a derecha
                            neighbours.append((point[0] + 1, point[1], self._action_costs[3]))
                    if d_y > 0:
                        #Se movía disminuyendo y
                        if new_direction[0] > 0:
                            #Giro a derecha
                            neighbours.append((point[0] + 1, point[1], self._action_costs[3]))
                        if new_direction[0] < 0:
                            #Giro a izquierda
                            neighbours.append((point[0] + 1, point[1], self._action_costs[2]))
                if d_y == 0 and d_x != 0:
                    #Se movía en horizontal
                    if d_x > 0:
                        #Se movía aumentando x
                        if new_direction[1] > 0:
                            #Giro a izquierda
                            neighbours.append((point[0] + 1, point[1], self._action_costs[2]))
                        if new_direction[1] < 0:
                            #Giro a derecha
                            neighbours.append((point[0] + 1, point[1], self._action_costs[3]))
                    if d_x < 0:
                        #Se movía aumentando x
                        if new_direction[1] > 0:
                            #Giro a derecha
                            neighbours.append((point[0] + 1, point[1], self._action_costs[3]))
                        if new_direction[1] < 0:
                            #Giro a izquierda
                            neighbours.append((point[0] + 1, point[1], self._action_costs[2]))
                else:
                    neighbours.append((point[0] + 1, point[1], 1))  # FIRST STEP

            new_direction = [0, 1] # BOTTOM
            a = direction[0] * new_direction[0] + direction[1] * new_direction[1]
            c = np.dot(np.asanyarray(direction), np.asanyarray(new_direction))  # -> cosine of the angle

            if c == 1:
                neighbours.append((point[0], point[1] + 1, self._action_costs[0]))  # STRAIGHT
            elif c == -1:
                neighbours.append((point[0], point[1] + 1, self._action_costs[1]))  # BACKWARDS
            elif c == 0:
                if d_x == 0 and d_y != 0:
                    #Se movía en vertical
                    if d_y > 0:
                        #Se movía aumentando y
                        if new_direction[0] > 0:
                            #Giro a izquierda
                            neighbours.append((point[0], point[1] + 1, self._action_costs[2]))
                        if new_direction[0] < 0:
                            #Giro a derecha
                            neighbours.append((point[0], point[1] + 1, self._action_costs[3]))
                    if d_y > 0:
                        #Se movía disminuyendo y
                        if new_direction[0] > 0:
                            #Giro a derecha
                            neighbours.append((point[0], point[1] + 1, self._action_costs[3]))
                        if new_direction[0] < 0:
                            #Giro a izquierda
                            neighbours.append((point[0], point[1] + 1, self._action_costs[2]))
                if d_y == 0 and d_x != 0:
                    #Se movía en horizontal
                    if d_x > 0:
                        #Se movía aumentando x
                        if new_direction[1] > 0:
                            #Giro a izquierda
                            neighbours.append((point[0], point[1] + 1, self._action_costs[2]))
                        if new_direction[1] < 0:
                            #Giro a derecha
                            neighbours.append((point[0], point[1] + 1, self._action_costs[3]))
                    if d_x < 0:
                        #Se movía aumentando x
                        if new_direction[1] > 0:
                            #Giro a derecha
                            neighbours.append((point[0], point[1] + 1, self._action_costs[3]))
                        if new_direction[1] < 0:
                            #Giro a izquierda
                            neighbours.append((point[0], point[1] + 1, self._action_costs[2]))
                else:
                    neighbours.append((point[0], point[1] + 1, 1))  # FIRST STEP

            new_direction = [0, -1] #TOP
            a = direction[0] * new_direction[0] + direction[1] * new_direction[1]
            c = np.dot(np.asanyarray(direction), np.asanyarray(new_direction))  # -> cosine of the angle

            if c == 1:
                neighbours.append((point[0], point[1] - 1, self._action_costs[0]))  # STRAIGHT
            elif c == -1:
                neighbours.append((point[0], point[1] - 1, self._action_costs[1]))  # BACKWARDS
            elif c == 0:
                if d_x == 0 and d_y != 0:
                    #Se movía en vertical
                    if d_y > 0:
                        #Se movía aumentando y
                        if new_direction[0] > 0:
                            #Giro a izquierda
                            neighbours.append((point[0], point[1] - 1, self._action_costs[2]))
                        if new_direction[0] < 0:
                            #Giro a derecha
                            neighbours.append((point[0], point[1] - 1, self._action_costs[3]))
                    if d_y > 0:
                        #Se movía disminuyendo y
                        if new_direction[0] > 0:
                            #Giro a derecha
                            neighbours.append((point[0], point[1] - 1, self._action_costs[3]))
                        if new_direction[0] < 0:
                            #Giro a izquierda
                            neighbours.append((point[0], point[1] - 1, self._action_costs[2]))
                if d_y == 0 and d_x != 0:
                    #Se movía en horizontal
                    if d_x > 0:
                        #Se movía aumentando x
                        if new_direction[1] > 0:
                            #Giro a izquierda
                            neighbours.append((point[0], point[1] - 1, self._action_costs[2]))
                        if new_direction[1] < 0:
                            #Giro a derecha
                            neighbours.append((point[0], point[1] - 1, self._action_costs[3]))
                    if d_x < 0:
                        #Se movía aumentando x
                        if new_direction[1] > 0:
                            #Giro a derecha
                            neighbours.append((point[0], point[1] - 1, self._action_costs[3]))
                        if new_direction[1] < 0:
                            #Giro a izquierda
                            neighbours.append((point[0], point[1] - 1, self._action_costs[2]))
                else:
                    neighbours.append((point[0], point[1] - 1, 1))  # FIRST STEP


            for neighbour in neighbours:
                try:
                    if self._map.grid_map[neighbour[0], neighbour[1]] != 1 and np.sign(neighbour[0]) != -1 and np.sign(neighbour[1]) != -1 and neighbour[0] < map_rows  and neighbour [1] < map_cols:
                        g = point[3] + neighbour[2]
                        if matrix_appended[neighbour[0], neighbour[1]] is None or matrix_appended[neighbour[0], neighbour[1]][2] > g:
                            open_list.append((neighbour[0], neighbour[1], heuristic[neighbour[0], neighbour[1]] + g, g))
                            matrix_appended[neighbour[0],neighbour[1]] = (point[0], point[1], g)
                except:
                    print("Point not in map")

            neighbours.clear()

            open_list.sort(key = lambda r: (r[2]))

            point = open_list[0]

            closed_list.append(open_list[0][:2])

            print("Number of steps: " + str(i))

            i += 1

            del open_list[0]


        reconstruct_path = self._reconstruct_path(start_rc, goal_rc, ancestors = matrix_appended)

        reconstruct_path.reverse()

        return reconstruct_path

    @staticmethod
    def smooth_path(path, data_weight: float = 0.5, smooth_weight: float = 0.1, tolerance: float = 1e-9) -> \
            List[Tuple[float, float]]:

        """Computes a smooth trajectory from a Manhattan-like path.

        Args:
            path: Non-smoothed path to the goal (start location first).
            data_weight: The larger, the more similar the output will be to the original path.
            smooth_weight: The larger, the smoother the output path will be.
            tolerance: The algorithm will stop when after an iteration the smoothed path changes less than this value.

        Returns: Smoothed path (initial location first) in (x, y) format.

        """

        new_path = [[0 for col in range(len(path[0]))] for row in range(len(path))]

        for i in range(len(path)):
            for j in range(len(path[0])):
                new_path[i][j] = path[i][j]

        change = 1
        while change > tolerance:
            change = 0
            for i in range(1, len(path) - 1):
                for j in range(len(path[0])):

                    old_path = new_path[i][j]
                    new_path[i][j] = new_path[i][j] + data_weight * (path[i][j] - new_path[i][j])
                    new_path[i][j] = new_path[i][j] + smooth_weight * (new_path[i + 1][j] + new_path[i - 1][j] - 2 * new_path[i][j])
                    change += abs(old_path - new_path[i][j])

        return new_path

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
        map_rows, map_cols = np.shape(self._map.grid_map)


        heuristic = np.ndarray(shape = (map_rows, map_cols), dtype = int)

        goal_rc = self._xy_to_rc(goal)

        for i in range(0, map_rows):
            for j in range (0, map_cols):
                heuristic[i][j] = abs(goal_rc[0] - i) + abs(goal_rc[1] - j)

        return heuristic

    def _reconstruct_path(self, start: Tuple[float, float], goal: Tuple[float, float], ancestors: np.ndarray) -> \
            List[Tuple[float, float]]:
        """Computes the trajectory from the start to the goal location given the ancestors of a search algorithm.

        Args:
            start: Initial location in (x, y) format.
            goal: Goal location in (x, y) format.
            ancestors: Matrix that contains for every cell, None or the (x, y) ancestor from which it was opened.

        Returns: Path to the goal (start location first) in (x, y) format.

        """
        path = []
        current = goal

        path.append(self._rc_to_xy(goal))

        while current != start:
            current = (ancestors[current[0], current[1]][0], ancestors[current[0], current[1]][1])
            path.append(self._rc_to_xy(current))

        return path

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
    action_costs = (1.0, 1.0, 1.0, 1.0) #Straight, Back, Turn Left, Turn Right

    planning = Planning(m, action_costs)
    path = planning.a_star(start, goal)
    smoothed_path = planning.smooth_path(path, data_weight=0.1, smooth_weight=0.05)
    planning.show(path, smoothed_path, blocking=True)


if __name__ == '__main__':
    test()
