from typing import List, Tuple
import numpy as np

import math


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
            error = 2 * ((measurements[6-1]) + (measurements[7-1]) + (measurements[8-1]) - 0.45 - 0.45 - 0.5)
            # print("Lado izquierdo vacío")
        elif ((measurements[7 - 1] > 0.9 and measurements[8 - 1] > 0.9) and (measurements[1-1] < 0.8 and measurements[2-1] < 0.8) and measurements[4-1] > 0.9 and measurements[5-1] > 0.9):
            error = -2 * ((measurements[1 - 1]) + (measurements[2 - 1]) + (measurements[3 - 1]) - 0.45 - 0.45 - 0.5)
            # print("Lado derecho vacío")
        else:
            error = - 1 * (measurements[1-1]) - (measurements[2-1]) - (measurements[3-1]) + (measurements[6-1]) + (measurements[7-1]) + (measurements[8-1]) #La mayor parte del tiempo

        error_acumulation.record(error) #Error acumulation es un buffer circular

        # print(str(measurements[1-1]) + "," + str(measurements[2-1]) + "," + str(measurements[3-1]) + "," + str(measurements[4-1]) + "," + str(measurements[5-1]) + "," + str(measurements[6-1]) + "," + str(measurements[7-1]) + "," + str(measurements[8-1]))

        der = (error - error_acumulation.__getitem__(error_acumulation._index - 2)) / 0.05

        integ = sum(error_acumulation.get_all()) * 0.05


        if math.fabs(integ) < 0.15:
            v = 0.3
            w = - 0.8 * integ
        else:
            v = 0.1 * integ
            w = - 3 * integ

        if ((measurements[4-1] - 1) + (measurements[5-1] - 1)) < -1.2 or (measurements[4] - 1)< -0.7 or (measurements[5-1] - 1) < -0.7:
            v = 0
            w = -1

        return v, w


    def explore2(self, measurements: List[float], error_acumulation) -> Tuple[float, float]:
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
            error = 2 * ((measurements[6-1]) + (measurements[7-1]) + (measurements[8-1]) - 0.45 - 0.45 - 0.5)
            print("Lado izquierdo vacío")
        elif ((measurements[7 - 1] > 0.9 and measurements[8 - 1] > 0.9) and (measurements[1-1] < 0.8 and measurements[2-1] < 0.8) and measurements[4-1] > 0.9 and measurements[5-1] > 0.9):
            error = -2 * ((measurements[1 - 1]) + (measurements[2 - 1]) + (measurements[3 - 1]) - 0.45 - 0.45 - 0.5)
            print("Lado derecho vacío")
        else:
            error = - 1 * (measurements[1-1]) - (measurements[2-1]) - (measurements[3-1]) + (measurements[6-1]) + (measurements[7-1]) + (measurements[8-1]) #La mayor parte del tiempo

        error_acumulation.record(error) #Error acumulation es un buffer circular

        # print(str(measurements[1-1]) + "," + str(measurements[2-1]) + "," + str(measurements[3-1]) + "," + str(measurements[4-1]) + "," + str(measurements[5-1]) + "," + str(measurements[6-1]) + "," + str(measurements[7-1]) + "," + str(measurements[8-1]))

        der = (error - error_acumulation.__getitem__(error_acumulation._index - 2)) / 0.05

        integ = sum(error_acumulation.get_all()) * 0.05

        v = 0.3
        w = - 0.7 * integ

        if w > 1:
            w = 0

        if ((measurements[4 - 1] - 1) + (measurements[5 - 1] - 1)) < -1.8:
            v = 0

        return v, w
