import vrep

from idle import Idle
from robot_p3dx import RobotP3DX
from navigation import Navigation

from circularbuffer import CircularBuffer

from particle_filter import *

from planning import *

from kmeans import *

from _functools import reduce

import matplotlib

import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')


import time

from map import Map
from typing import List, Tuple


if __name__ == '__main__':
    # Connect to V-REP
    vrep.simxFinish(-1)  # Close all pending connections
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 1000, 5)

    if client_id == -1:
        raise ConnectionError('Could not connect to V-REP. Make sure the simulator is running and that \n'
                              'the following command is included in the initialization of a child script:\n'
                              'simRemoteApi.start(19999)')

    # Write initialization code here
    ts = 0.13
    robot = RobotP3DX(client_id)
    navigation = Navigation()
    idle = Idle(ts)

    error_acumulation = CircularBuffer(3) #4 es factible, 5 y 2 no funcionan con estos parámetros

    for error in error_acumulation:
        error_acumulation.record(0)

    particle_count = 500

    m = Map('map_project.json')
    pf = ParticleFilter(m, RobotP3DX.SENSORS[:8], RobotP3DX.SENSOR_RANGE, particle_count = particle_count)

    dt = 1

    count = 0

    found = 0

    try:
        while True:
            while found == 0:
                # Write your control algorithm here
                start = time.time()
                z = robot.sense()
                v, w = navigation.explore(z, error_acumulation)
                loop_time, overflow = idle.task()
                timer = time.time() - start
                # print("Measurements:" + str(timer))

                print("COUNT: " + str(count))

                if count % 40 == 0 and count != 0 or count == 450:
                    robot.move(0,0)
                    pf.move(0,0,0)
                    start = time.time()
                    pf.resample(z[:8])
                    sense = time.time() - start
                    # Display timing results
                    print('Total: {0:6.3f} s     Move: {1:6.3f} s     Sense: {2:6.3f} s'.format(move + sense, move, sense))
                    # pf.show(1, 'Move', save_figure=True)

                # Move
                if overflow is not True:
                    start = time.time()
                    pf.move(v / 1.6, w / 1.6, ts)
                    robot.move(v / 1.6, w / 1.6)
                    move = time.time() - start
                else:
                    robot.move(0,0)
                    pf.move(0,0,0)

                # print("Particle Move:" + str(move))


                count += 1

                start = time.time()
                # pf.show(1, 'Move', save_figure=True)
                show = time.time() - start
                # print("Show" + str(show))

                if overflow:
                    print('Loop time: {0:.3f} s'.format(loop_time))

                if count == 450:
                    print("DONNEEEEEEE")
                    found = 1
                    points = []
                    for i in range (0,len(pf._particles)):
                        a = Point(pf._particles[i][:2])
                        points.append(a)
                    clusters, iterations = kmeans(points, 2, 0.3)

                    num = 0
                    for c in clusters:
                        if len(c.points) > num:
                            num = len(c.points)
                            centroid = c.centroid
                            print(centroid.coords)
                            print(len(c.points))


                    start_point = (centroid.coords[0],centroid.coords[1])
                    print(start_point)


            while found == 1:
                # start = (4,-4)
                robot.move(0,0)
                goal = (4, 4)
                action_costs = (1.0, 1.0, 1.0, 1.0)  # Straight, Back, Turn Left, Turn Right
                planning = Planning(m, action_costs, naive=False)
                path = planning.a_star(start_point, goal)
                smoothed_path = planning.smooth_path(path, data_weight=0.8, smooth_weight=0.2)
                planning.show(path, smoothed_path, blocking=True)
                print(smoothed_path)



    except KeyboardInterrupt:  # Break the infinite loop to gracefully close the connection
        pass

    # Close the connection
    vrep.simxGetPingTime(client_id)  # Make sure that the last command sent out had time to arrive
    vrep.simxFinish(client_id)
