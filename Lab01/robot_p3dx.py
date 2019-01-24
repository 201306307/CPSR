import numpy as np
import vrep

from robot import Robot
from typing import Any, Dict, List, Tuple
from vrep import *
from vrepConst import *
from numpy import *


class RobotP3DX(Robot):
    """Class to control the Pioneer 3-DX robot."""

    # Constants
    SENSOR_RANGE = 1       # Ultrasonic sensor range [m]
    TRACK = 0.33           # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.0975  # Radius of the wheels [m]

    # Sensor location and orientation (x, y, theta) in the robot coordinate frame
    SENSORS = [(0.1067, 0.1382, 1.5708),
               (0.1557, 0.1250, 0.8727),
               (0.1909, 0.0831, 0.5236),
               (0.2095, 0.0273, 0.1745),
               (0.2095, -0.0273, -0.1745),
               (0.1909, -0.0785, -0.5236),
               (0.1558, -0.1203, -0.8727),
               (0.1067, -0.1382, -1.5708),
               (-0.1100, -0.1382, -1.5708),
               (-0.1593, -0.1203, -2.2689),
               (-0.1943, -0.0785, -2.6180),
               (-0.2129, -0.0273, -2.9671),
               (-0.2129, 0.0273, 2.9671),
               (-0.1943, 0.0785, 2.6180),
               (-0.1593, 0.1203, 2.2689),
               (-0.1100, 0.1382, 1.5708)]

    def __init__(self, client_id: int):
        """Pioneer 3-DX robot class initializer.

        Args:
            client_id: V-REP connection handle.

        """
        Robot.__init__(self, client_id, track=self.TRACK, wheel_radius=self.WHEEL_RADIUS)
        self._motors = self._init_motors()
        self._sensors = self._init_sensors()

    def move(self, v, w) -> Tuple[float, float]:
        """Solve inverse differential kinematics and send commands to the motors.

        Args:
            v: Linear velocity of the robot center [m/s].
            w: Angular velocity of the robot center [rad/s].

        """
        # TODO: Complete with your code.

        left_set = simxSetJointTargetVelocity(self._client_id, self._motors["left"], w, simx_opmode_oneshot)
        right_set = simxSetJointTargetVelocity(self._client_id, self._motors["right"], w, simx_opmode_oneshot)

        return (left_set, right_set)

    def sense(self) -> List[float]:
        """Read ultrasonic sensors.

        Returns: Distance from every sensor to the closest obstacle [m].

        """
        # TODO: Complete with your code.

        pass

    def _init_motors(self) -> Dict[str, int]:
        """Acquire motor handles.

        Returns: {'left': handle, 'right': handle}

        """
        rcleft, leftMotor = simxGetObjectHandle(self._client_id,"Pioneer_p3dx_leftMotor",simx_opmode_blocking) #Handle of the left motor
        rcright, rightMotor = simxGetObjectHandle(self._client_id,"Pioneer_p3dx_rightMotor",simx_opmode_blocking) #Handle of the rightMotor

        motors = {"left": leftMotor,
                "right": rightMotor}

        return motors

    def _init_sensors(self) -> List[Any]:
        """Acquire sensor handles and initialize streaming.

        Returns: List with sensor handles.

        """
        # TODO: Complete with your code.

        pass
