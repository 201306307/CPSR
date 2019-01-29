from typing import List, Tuple
import numpy as np


from circularbuffer import CircularBuffer



class Navigation:
    """Class for short-term path planning."""

    def __init__(self):
        """Navigation class initializer."""
        # TODO: Add code if necessary.
        pass

    def explore(self, measurements: List[float], error_acumulation) -> Tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            measurements: Distance from every sensor to the closest obstacle [m].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: Compute v and w with your algorithm
        #PONGO MENOS 1 PORQUE EL SENSOR 1 ESTA EN LA POSICION 0, EL 2 EN LA 1 ETC

        if ((measurements[1-1] > 0.9 and measurements[2-1] > 0.9) and (measurements[7-1] < 0.8 and measurements[8-1] < 0.8) and measurements[4-1] > 0.9 and measurements[5-1] > 0.9):
            error = 3 * ((measurements[6-1]) + (measurements[7-1]) + (measurements[8-1]) - 0.45 - 0.45 - 0.5)
            print("Lado izquierdo vacío")
        elif ((measurements[7 - 1] > 0.9 and measurements[8 - 1] > 0.9) and (measurements[1-1] < 0.8 and measurements[2-1] < 0.8) and measurements[4-1] > 0.9 and measurements[5-1] > 0.9):
            error = -3 * ((measurements[1 - 1]) + (measurements[2 - 1]) + (measurements[3 - 1]) - 0.45 - 0.45 - 0.5)
            print("Lado derecho vacío")
        else:
            error = - 1 * (measurements[1-1]) - (measurements[2-1]) - (measurements[3-1]) + (measurements[6-1]) + (measurements[7-1]) + (measurements[8-1]) #La mayor parte del tiempo

        error_acumulation.record(error) #Error acumulation es un buffer circular

        # print(str(measurements[1-1]) + "," + str(measurements[2-1]) + "," + str(measurements[3-1]) + "," + str(measurements[4-1]) + "," + str(measurements[5-1]) + "," + str(measurements[6-1]) + "," + str(measurements[7-1]) + "," + str(measurements[8-1]))

        der = (error - error_acumulation.__getitem__(error_acumulation._index - 2)) / 0.05

        int = sum(error_acumulation.get_all()) * 0.05

        k = 0.3 / error

        if np.abs(int) < 0.18:
            v = k * error
            w = - int
        else:
            v = 0.1 * int
            w = - 1.8 * int

        if ((measurements[4-1] - 1) + (measurements[5-1] - 1)) < -1.5:
            v = 0
            w = -20 * int

        return v, w
