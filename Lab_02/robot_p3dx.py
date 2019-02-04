import numpy as np
import vrep

from robot import Robot
from typing import Any, Dict, List, Tuple


class RobotP3DX(Robot):
    """Class to control the Pioneer 3-DX robot."""

    # Constants
    SENSOR_RANGE = 1      # Ultrasonic sensor range [m]
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
        self._move_counter = 0

    def move(self, v, w) -> Tuple[float, float]:
        """Solve inverse differential kinematics and send commands to the motors.

        Args:
            v: Linear velocity of the robot center [m/s].
            w: Angular velocity of the robot center [rad/s].

        Returns:
            wr: Right wheel angular velocity [rad/s].
            wl: Left wheel angular velocity [rad/s].

        """
        # Solve inverse differential kinematics
        wr = (v + w * self._track / 2) / self._wheel_radius
        wl = (v - w * self._track / 2) / self._wheel_radius

        # Set motor speeds
        # Communication is disabled to ensure the commands are applied almost simultaneously
        vrep.simxPauseCommunication(self._client_id, True)
        if self._move_counter % 2:  # Change command order alternatively to reduce rotation drift
            vrep.simxSetJointTargetVelocity(self._client_id, self._motors.get('left'), wl, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(self._client_id, self._motors.get('right'), wr, vrep.simx_opmode_oneshot)
        else:
            vrep.simxSetJointTargetVelocity(self._client_id, self._motors.get('right'), wr, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(self._client_id, self._motors.get('left'), wl, vrep.simx_opmode_oneshot)
        vrep.simxPauseCommunication(self._client_id, False)

        self._move_counter += 1

        return wr, wl

    def sense(self) -> List[float]:
        """Read ultrasonic sensors.

        Returns: Distance from every sensor to the closest obstacle [m].

        """
        measurements = [float('inf')] * len(self.SENSORS)

        for i in range(0, len(measurements)):
            _, is_valid, detected_point, _, _ = vrep.simxReadProximitySensor(self._client_id, self._sensors[i], vrep.simx_opmode_buffer)

            if is_valid:
                measurements[i] = np.linalg.norm(detected_point)

        return measurements

    def _init_motors(self) -> Dict[str, int]:
        """Acquire motor handles.

        Returns: {'left': handle, 'right': handle}

        """
        motors = {'left': None, 'right': None}

        for motor in motors:
            rc, motors[motor] = vrep.simxGetObjectHandle(self._client_id, 'Pioneer_p3dx_' + motor + 'Motor', vrep.simx_opmode_blocking)

            if rc != vrep.simx_return_ok:
                raise ValueError('vrep.simxGetObjectHandle returned with error code: ' + str(rc))

        return motors

    def _init_sensors(self) -> List[Any]:
        """Acquire sensor handles and initialize streaming.

        Returns: List with sensor handles.

        """
        sensors = [None] * len(self.SENSORS)

        for i in range(0, len(sensors)):
            rc, sensors[i] = vrep.simxGetObjectHandle(self._client_id, 'Pioneer_p3dx_ultrasonicSensor' + str(i + 1), vrep.simx_opmode_blocking)

            if rc != vrep.simx_return_ok:
                raise ValueError('vrep.simxGetObjectHandle returned with error code: ' + str(rc))

            vrep.simxReadProximitySensor(self._client_id, sensors[i], vrep.simx_opmode_streaming)

        return sensors
