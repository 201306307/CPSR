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
        #PONGO MENOS 1 PORQUE EL SENSOR 1 ESTA EN LA POSICION 0, EL 2 EN LA 1 ETC

        if measurements[4-1] > 0.4 and measurements[5-1] > 0.4 and measurements[3-1] > 0.3 and measurements[6-1] > 0.3:
            #CONTROL EN RECTA
            if measurements[8-1] < 0.3:
                print("alejo de derecha")
                w = 0.25
                v = 1
            if measurements[16-1] < 0.3:
                print("alejo de izquierda")
                w = -0.25
                v = 1
            else:
                print("recta")
                w = 0
                v = 2
        else:
            #CONTROL EN CURVA
            (if measurements[16-1] > 0.99 and [1-1] > 0.99) or (measurements[8-1] < 0.5 and measurements[9-1] < 0.5):
                w = 0.25
                v = 0
                print("curva seguir pared giro hacia izq")
            else:
                w = -0.25
                v = 0
                print("curva seguir pared giro hacia derecha")


        return v, w
