import json
import math
import numpy as np

from intersect import Intersect
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from typing import List, Tuple

import matplotlib
matplotlib.use("TkAgg")  # Prevents matplotlib from crashing in macOS
from matplotlib import pyplot as plt


class Map:
    """Class to perform operations on metric maps."""

    def __init__(self, json_file: str):
        """Map class initializer.

        Args:
            json_file: JSON file with the coordinates of the external boundary and the internal hole vertices.

        """
        # Load map from JSON file
        with open(json_file, "r") as read_file:
            data = json.load(read_file)

        boundary = data['boundary']
        holes = data['holes']

        # Create a polygon map
        self._map_polygon = Polygon(boundary, holes=holes)

        # Create a segment map
        self._map_segments = []

        for i in range(len(boundary) - 1):
            self._map_segments.append([boundary[i], boundary[i + 1]])

        self._map_segments.append([boundary[-1], boundary[0]])

        for hole in holes:
            for i in range(len(hole) - 1):
                self._map_segments.append([hole[i], hole[i + 1]])

            self._map_segments.append([hole[-1], hole[0]])

    def bounds(self) -> Tuple[float, float, float, float]:
        """Coordinates of a bounding box that contains the map.

        Returns:
            x_min: Bottom left corner x coordinate [m].
            y_min: Bottom left corner y coordinate [m].
            x_max: Top right corner x coordinate [m].
            y_max: Top right corner y coordinate [m].

        """
        return self._map_polygon.bounds

    def check_collision(self, segment: List[Tuple[float, float]], compute_distance: bool = False) -> \
            Tuple[Tuple[float, float], float]:
        """Determines if a segment intersects with the map.

        Args:
            segment: Sensor ray or motion trajectory in the format [(start), (end)].
            compute_distance: True to compute the distance between the robot and the intersection point.

        Returns:
            intersection: Closest collision point (x, y) [m].
            distance: Distance to the obstacle [m]. inf if not computed.

        """
        intersect = Intersect()

        intersections = []
        distance = float('inf')
        index = 0

        for map_segment in self._map_segments:
            pt = intersect.segment_intersect(segment, map_segment)

            if pt is not None:
                intersections.append(pt)

        if (compute_distance and intersections) or len(intersections) > 1:
            distances = [math.sqrt((pt[0] - segment[0][0]) ** 2 + (pt[1] - segment[0][1]) ** 2) for pt in intersections]
            index = int(np.argmin(distances))
            distance = distances[index]

        intersection = intersections[index] if intersections else []

        return intersection, distance

    def contains(self, point: Tuple[float, float]) -> bool:
        """Determines whether a point is within the map limits.

        Args:
            point: (x, y) coordinates to check.

        Returns:
            bool: True if the point is inside the map; False otherwise.

        Warnings:
            It might return False if the point is very close to a boundary.

        """
        pt = Point(round(point[0], 6), round(point[1], 6)).buffer(0.2)

        return self._map_polygon.contains(pt)

    def plot(self, axes):
        """Draws the map.

           Args:
               axes: Figure axes.

           Returns:
               axes: Modified axes.

        """
        x_min, y_min, x_max, y_max = self.bounds()

        major_ticks = np.arange(math.floor(min(x_min, y_min)), math.ceil(max(x_max, y_max)) + 0.01, 1)
        minor_ticks = np.arange(math.floor(min(x_min, y_min)), math.ceil(max(x_max, y_max)) + 0.01, 0.5)

        axes.set_xticks(major_ticks)
        axes.set_xticks(minor_ticks, minor=True)
        axes.set_yticks(major_ticks)
        axes.set_yticks(minor_ticks, minor=True)

        axes.set_xlim(math.floor(x_min), math.ceil(x_max))
        axes.set_ylim(math.floor(y_min), math.ceil(y_max))
        axes.grid(which='both', alpha=0.33, linestyle='dashed', zorder=1)
        axes.set(xlabel='x [m]', ylabel='y [m]')

        # Plot map
        x, y = self._map_polygon.exterior.xy
        axes.plot(x, y, color='black', alpha=1, linewidth=3, solid_capstyle='round', zorder=2)

        for interior in self._map_polygon.interiors:
            x, y = interior.xy
            axes.plot(x, y, color='black', alpha=1, linewidth=3, solid_capstyle='round', zorder=2)

        return axes

    def show(self, figure_number: int = 1, title: str = '', blocking: bool = False,
             figure_size: Tuple[float, float] = (7, 7)):
        """Displays the map in a figure.

        Args:
            figure_number: Any existing figure with the same value will be overwritten.
            title: Plot title.
            blocking: True to stop program execution until the figure window is closed.
            figure_size: Figure window dimensions.

        """
        figure = plt.figure(figure_number, figsize=figure_size)
        figure.clf()

        axes = figure.add_subplot(1, 1, 1)
        axes = self.plot(axes)

        axes.set_title(title)

        if not blocking:
            plt.ion()          # Activate interactive mode
            plt.show()
            plt.pause(0.0001)  # Wait for 0.1 ms or the figure wont be displayed
        else:
            plt.show()
