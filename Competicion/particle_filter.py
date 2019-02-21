import math
import numpy as np
import os
import random

import time

from math import pi

from line_profiler import LineProfiler


from map import Map
from typing import List, Tuple

import matplotlib
matplotlib.use("TkAgg")  # Prevents matplotlib from crashing in macOS
from matplotlib import pyplot as plt


class ParticleFilter:
    """Particle filter implementation."""

    def __init__(self, map_object: Map, sensors: List[Tuple[float, float, float]], sensor_range: float,
                 particle_count: int = 100, sense_noise: float = 0.5, v_noise: float = 0.05, w_noise: float = 0.05):
        """Particle filter class initializer.

        Args:
            map_object: Map of the environment.
            sensors: Robot sensors location [m] and orientation [rad] in the robot coordinate frame (x, y, theta).
            sensor_range: Sensor measurement range [m]
            particle_count: Number of particles.
            sense_noise: Measurement standard deviation [m].
            v_noise: Linear velocity standard deviation [m/s].
            w_noise: Angular velocity standard deviation [rad/s].

        """
        self._map = map_object
        self._sensors = sensors
        self._sense_noise = sense_noise
        self._sensor_range = sensor_range
        self._v_noise = v_noise
        self._w_noise = w_noise
        self._iteration = 0

        self._particles = self._init_particles(particle_count)
        self._ds, self._phi = self._init_sensor_polar_coordinates(sensors)

    def move(self, v: float, w: float, dt: float):
        """Performs a motion update on the particles.

        Args:
            v: Linear velocity [m].
            w: Angular velocity [rad/s].
            dt: Sampling time [s].

        """
        self._iteration += 1

        # TODO: Complete with your code.
        for particle in self._particles:
            v_noise = v + np.random.normal(0, self._v_noise)
            w_noise = w + np.random.normal(0, self._w_noise)

            x = particle[0] + v_noise * dt * math.cos(particle[2])
            y = particle[1] + v_noise * dt * math.sin(particle[2])
            th = particle[2] + w_noise * dt

            # Check th in range [0, 2 pi)
            th = th % (2 * pi)

            # intersection, _ = self._map.check_collision([(particle[0], particle[1]), (x, y)], False)
            # if intersection:
            #     x = intersection[0]
            #     y = intersection[1]

            particle[0] = x
            particle[1] = y
            particle[2] = th

        pass

    def resample(self, measurements: List[float]):
        """Samples a new set of set of particles using the resampling wheel method.

        Args:
            measurements: Sensor measurements [m].

        """
        # First obtain normalized weights
        beta = 0
        new_particles = np.empty((len(self._particles), 3), dtype=object)

        N = len(self._particles)

        weights = [self._measurement_probability(measurements, particle) for particle in self._particles]

        weights_total = np.sum(weights)
        weights_normalized = [w / weights_total for w in weights]

        index = int(np.random.uniform(0, N))
        weight_max = max(weights_normalized)

        for i in range(0, N):
            beta = beta + random.random() * 2.0 * weight_max
            while beta >= weights_normalized[index]:
                beta = beta - weights_normalized[index]
                index = (index + 1)
                if index == N:
                    index = 0
            new_particles[i] = self._particles[index]

        self._particles = new_particles

    def plot(self, axes, orientation: bool = True):
        """Draws particles.

        Args:
            axes: Figure axes.
            orientation: Draw particle orientation.

        Returns:
            axes: Modified axes.

        """
        axes.plot(self._particles[:, 0], self._particles[:, 1], 'bo', markersize=2)

        if orientation:
            arrow_length = 0.1

            for particle in self._particles:
                dx = arrow_length * math.cos(particle[2])
                dy = arrow_length * math.sin(particle[2])
                axes.arrow(particle[0], particle[1], dx, dy, fc='y', ec='y', lw=1)

        return axes

    def show(self, figure_number: int = 1, title: str = '', blocking: bool = False,
             figure_size: Tuple[float, float] = (7, 7), orientation: bool = True,
             save_figure: bool = False, save_dir: str = 'img', centroid: Tuple[float, float, float] = (0, 0, 0)):
        """Displays the current particle set on the map.

        Args:
            figure_number: Any existing figure with the same value will be overwritten.
            title: Plot title.
            blocking: True to stop program execution until the figure window is closed.
            figure_size: Figure window dimensions.
            orientation: Draw particle orientation.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.
            centroid: Coordinates of guessed centroid.

        """
        figure = plt.figure(figure_number, figsize=figure_size)
        figure.clf()

        axes = figure.add_subplot(1, 1, 1)
        axes = self._map.plot(axes)
        axes = self.plot(axes, orientation)
        axes.plot(centroid[0], centroid[1], 'ro', markersize=4)
        dx = 0.5 * math.cos(centroid[2])
        dy = 0.5 * math.sin(centroid[2])
        axes.arrow(centroid[0], centroid[1], dx, dy, fc='r', ec='r', lw=1)

        axes.set_title(title + ' (Iteration #' + str(self._iteration) + ')')
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

            file_name = str(self._iteration).zfill(4) + ' ' + title.lower() + '.png'
            file_path = os.path.join(save_dir, file_name)
            figure.savefig(file_path, box_inches='tight')

    def _init_particles(self, particle_count: int) -> np.ndarray:
        """Draws N random valid particles.

        The particles are guaranteed to be inside the map and
        can only have the following orientations [0, pi/2, pi, 3pi/2].

        Args:
            particle_count: Number of particles.

        Returns: A numpy array of tuples (x, y, theta).

        """


        particles = np.empty((particle_count, 3), dtype=object)

        map_bounds = self._map.bounds()  # retrieve the bounds of the map (rectangle containing the map)


        for particle in particles:
            is_valid = False
            while not is_valid:
                particle[0] = int(random.uniform(map_bounds[0], map_bounds[2]))
                particle[1] = int(random.uniform(map_bounds[1], map_bounds[3]))

                # the orientation has only 4 possible values
                is_valid = self._map.contains((particle[0], particle[1]))  # check if particle is in map

            particle[2] = random.choice([0, np.pi / 2, np.pi, 3 * np.pi / 2])

        return particles

    @staticmethod
    def _init_sensor_polar_coordinates(sensors: List[Tuple[float, float, float]]) -> Tuple[List[float], List[float]]:
        """Converts the robots sensor location and orientation to polar coordinates wrt to the robot's coordinate frame.

        Args:
            sensors: Robot sensors location [m] and orientation [rad] (x, y, theta).

        Return:
            ds: List of magnitudes [m].
            phi: List of angles [rad].

        """
        ds = [math.sqrt(sensor[0] ** 2 + sensor[1] ** 2) for sensor in sensors]
        phi = [math.atan2(sensor[1], sensor[0]) for sensor in sensors]

        return ds, phi

    def _sense(self, particle: Tuple[float, float, float]) -> List[float]:
        """Obtains the predicted measurement of every sensor given the robot's location.

        Args:
            particle: Particle pose (x, y, theta) in [m] and [rad].

        Returns: List of predicted measurements; inf if a sensor is out of range.

        """

        rays = self._sensor_rays(particle)

        pred_measurements = []

        # TODO: Complete with your code.
        for ray in rays:
            _, distance = self._map.check_collision(ray, True)
            pred_measurements.append(distance)

        return pred_measurements


    @staticmethod
    def _gaussian(mu: float, sigma: float, x: float) -> float:
        """Computes the value of a Gaussian.

        Args:
            mu: Mean. - medida
            sigma: Standard deviation. - ruido
            x: Variable. - estimada

        Returns:
            float: Gaussian.

        """
        value = math.exp(- 0.5 * ((x - mu) / sigma) **2)

        return value


    def _measurement_probability(self, measurements: List[float], particle: Tuple[float, float, float]) -> float:
        """Computes the probability of a set of measurements given a particle's pose.

        If a measurement is unavailable (usually because it is out of range), it is replaced with twice the sensor range
        to perform the computation. This value has experimentally been proven valid to deal with missing measurements.
        Nevertheless, it might not be the optimal replacement value.

        Args:
            measurements: Sensor measurements [m].
            particle: Particle pose (x, y, theta) in [m] and [rad].

        Returns:
            float: Probability.

        """
        probability = 1

        measurements = [1 if math.isinf(z) else z for z in measurements]

        pred_measurements = self._sense(particle) #Predicted measurements


        pred_measurements = [measure if not measure == float('inf') else 1 for measure in
                             pred_measurements]

        for measure, pred_measure in zip(measurements, pred_measurements):
            probability *= self._gaussian(measure, self._sense_noise, pred_measure) #Compute weight

        return probability


    def _sensor_rays(self, particle: Tuple[float, float, float]) -> List[List[Tuple[float, float]]]:
        """Determines the simulated sensor ray segments for a given particle.

        Args:
            particle: Particle pose (x, y, theta) in [m] and [rad].

        Returns: Ray segments.
                 Format: [[(x0_begin, y0_begin), (x0_end, y0_end)], [(x1_begin, y1_begin), (x1_end, y1_end)], ...]

        """
        x = particle[0]
        y = particle[1]
        theta = particle[2]

        # Convert sensors to world coordinates
        xw = [x + ds * math.cos(theta + phi) for ds, phi in zip(self._ds, self._phi)]
        yw = [y + ds * math.sin(theta + phi) for ds, phi in zip(self._ds, self._phi)]
        tw = [sensor[2] for sensor in self._sensors]

        rays = []

        for xs, ys, ts in zip(xw, yw, tw):
            x_end = xs + self._sensor_range * math.cos(theta + ts)
            y_end = ys + self._sensor_range * math.sin(theta + ts)
            rays.append([(xs, ys), (x_end, y_end)])

        return rays


def test(measurements):
    """Function used to test the ParticleFilter class independently."""
    import time
    from robot_p3dx import RobotP3DX

    # Measurements from sensors 1 to 8 [m]
    measurements = [
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.9343, float('inf'), float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, float('inf'), float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, float('inf'), float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.8582, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.7066, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.5549, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), float('inf'), 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), float('inf'), 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), float('inf'), 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), float('inf'), 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.9920, float('inf'), float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8795, float('inf'), float('inf')),
        (0.3832, 0.6021, float('inf'), float('inf'), 1.2914, 0.9590, float('inf'), float('inf')),
        (0.4207, 0.7867, float('inf'), float('inf'), 0.9038, float('inf'), float('inf'), 0.5420),
        (0.4778, float('inf'), float('inf'), float('inf'), 0.8626, float('inf'), float('inf'), 0.3648),
        (0.5609, float('inf'), float('inf'), 0.9514, 0.9707, float('inf'), float('inf'), 0.3669),
        (0.6263, float('inf'), float('inf'), 0.8171, 0.8584, float('inf'), float('inf'), 0.4199),
        (0.6918, float('inf'), 0.9942, 0.6828, 0.7461, float('inf'), float('inf'), 0.5652),
        (0.7572, 0.9544, 0.9130, 0.5485, 0.6338, float('inf'), float('inf'), 0.7106),
        (0.8226, 0.8701, 0.8319, 0.4142, 0.5215, float('inf'), float('inf'), 0.8559),
        (0.8880, 0.7858, 0.7507, 0.2894, 0.4092, float('inf'), float('inf'), float('inf')),
        (0.9534, 0.7016, 0.6696, 0.2009, 0.2969, float('inf'), float('inf'), float('inf')),
        (float('inf'), 0.6173, 0.5884, 0.1124, 0.1847, 0.4020, float('inf'), float('inf')),
        (0.9789, 0.5330, 0.1040, 0.0238, 0.0724, 0.2183, float('inf'), float('inf'))]

    # Wheel angular speed commands (left, right) [rad/s]
    motions = [(0, 0), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1),
               (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (0.5, 0), (0.5, 0), (0.5, 0), (0.5, 0),
               (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1)]

    dt = 1  # Sampling time [s]

    m = Map('map_pf.json')
    pf = ParticleFilter(m, RobotP3DX.SENSORS[:8], RobotP3DX.SENSOR_RANGE, particle_count=500)

    for u, z in zip(motions, measurements):
        # Solve differential kinematics
        v = (u[0] + u[1]) * RobotP3DX.WHEEL_RADIUS / 2
        w = (u[1] - u[0]) * RobotP3DX.WHEEL_RADIUS / RobotP3DX.TRACK

        # Move
        start = time.time()
        pf.move(v, w, dt)
        move = time.time() - start

        pf.show(1, 'Move', save_figure=True)

        # Sense
        start = time.time()
        pf.resample(z)
        sense = time.time() - start

        pf.show(1, 'Sense', save_figure=True)

        # Display timing results
        print('Total: {0:6.3f} s     Move: {1:6.3f} s     Sense: {2:6.3f} s'.format(move + sense, move, sense))


# This "strange" function is only called if this script (particle_filter.py) is the program's entry point.
if __name__ == '__main__':
    test()
