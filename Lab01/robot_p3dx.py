import numpy as np
import vrep

from robot import Robot
from typing import Any, Dict, List, Tuple


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
        self._client_id = client_id

    def move(self, v, w) -> Tuple[float, float]:
        """Solve inverse differential kinematics and send commands to the motors.

        Args:
            v: Linear velocity of the robot center [m/s].
            w: Angular velocity of the robot center [rad/s].

        """
        rc = vrep.simxPauseCommunication(self._client_id, True)
        """
        Con distintas velocidades en cada rueda, el robot gira
        Con w > v y w > 0, gira en sentido contrario a las agujas del reloj
        """
        rc = vrep.simxSetJointTargetVelocity(self._client_id, self._motors['right'], (v + self.TRACK/2 * w) / self.WHEEL_RADIUS, vrep.simx_opmode_oneshot)
        rc = vrep.simxSetJointTargetVelocity(self._client_id, self._motors['left'], (v - self.TRACK/2 * w) / self.WHEEL_RADIUS, vrep.simx_opmode_oneshot)
        rc = vrep.simxPauseCommunication(self._client_id, False)

    def sense(self) -> List[float]:
        """Read ultrasonic sensors.

        Returns: Distance from every sensor to the closest obstacle [m].

        """
        distance = []
        for i in range (0,16):
            rc, is_valid, detected_point, _, _ = vrep.simxReadProximitySensor(self._client_id, self._sensors[i], vrep.simx_opmode_buffer)


            if is_valid:
                distance.append(np.linalg.norm(detected_point))

            else:
                distance.append(1) #Poner inf u otro valor grande desvirtúa el cálculo del error, 1 funciona bien


        # print(distance)
        return distance

    def _init_motors(self) -> Dict[str, int]:
        """Acquire motor handles.

        Returns: {'left': handle, 'right': handle}

        """

        # Handles
        rc , handle_right = vrep.simxGetObjectHandle ( self._client_id , "Pioneer_p3dx_rightMotor" , vrep . simx_opmode_blocking )
        rc, handle_left = vrep.simxGetObjectHandle(self._client_id, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_blocking)


        motors = {"left": handle_left,
                  "right": handle_right}

        return motors

    def _init_sensors(self) -> List[Any]:
        """Acquire sensor handles and initialize streaming.


        Returns: List with sensor handles.

        """
        handle_sensor = []
        for i in range (1, 17):
            rc, handle = vrep.simxGetObjectHandle(self._client_id, "Pioneer_p3dx_ultrasonicSensor" + str(i) , vrep.simx_opmode_blocking)
            handle_sensor.append(handle)
            if rc != vrep.simx_return_ok:
                print("error")
            vrep.simxReadProximitySensor(self._client_id, handle, vrep.simx_opmode_streaming)

        return handle_sensor




