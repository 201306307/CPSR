import vrep

from idle import Idle
from robot_p3dx import RobotP3DX
from navigation import Navigation

from circularbuffer import CircularBuffer


if __name__ == '__main__':
    # Connect to V-REP
    vrep.simxFinish(-1)  # Close all pending connections
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 1000, 5)

    if client_id == -1:
        raise ConnectionError('Could not connect to V-REP. Make sure the simulator is running and that \n'
                              'the following command is included in the initialization of a child script:\n'
                              'simRemoteApi.start(19999)')

    # Write initialization code here
    ts = 0.05
    robot = RobotP3DX(client_id)
    navigation = Navigation()
    idle = Idle(ts)

    error_acumulation = CircularBuffer(3);

    for error in error_acumulation:
        error_acumulation.record(0)

    try:
        while True:
            # Write your control algorithm here
            z = robot.sense()
            v, w = navigation.explore(z, error_acumulation)
            robot.move(v, w)
            loop_time, overflow = idle.task()

            if overflow:
                print('Loop time: {0:.3f} s'.format(loop_time))

    except KeyboardInterrupt:  # Break the infinite loop to gracefully close the connection
        pass

    # Close the connection
    vrep.simxGetPingTime(client_id)  # Make sure that the last command sent out had time to arrive
    vrep.simxFinish(client_id)
