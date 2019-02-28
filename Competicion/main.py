import vrep

from idle import Idle
from robot_p3dx import RobotP3DX
from navigation import Navigation

from circularbuffer import CircularBuffer

from particle_filter import *

from planning import *

from kmeans import *

from clustering import *

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
    ts = 0.2
    robot = RobotP3DX(client_id)
    navigation = Navigation()
    idle = Idle(ts)

    error_acumulation = CircularBuffer(3) #4 es factible, 5 y 2 no funcionan con estos parámetros

    for error in error_acumulation:
        error_acumulation.record(0)

    particle_count = 950

    m = Map('map_project.json')
    pf = ParticleFilter(m, RobotP3DX.SENSORS[:16], RobotP3DX.SENSOR_RANGE, particle_count = particle_count)

    clustering = Clustering(pf)

    dt = 1

    count = 0

    found = 0

    time_giros = 25
    time_recta = 20

    counter_path = 0

    counting = 0

    angle_old = 0

    localized = False


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

                if count % 20 == 0 and count != 0:
                    robot.move(0,0)
                    pf.move(0,0,0)
                    start = time.time()
                    pf.resample(z[:16])
                    sense = time.time() - start
                    # Display timing results
                    # print('Total: {0:6.3f} s     Move: {1:6.3f} s     Sense: {2:6.3f} s'.format(move + sense, move, sense))
                    localized = clustering.localize(0.1)
                    coord = (0, 0, 0)

                if count % 100 == 0 and count != 0:
                    pf.show(1, 'Move', save_figure=True)


                # Move
                if overflow is not True:
                    start = time.time()
                    robot.move(v, w)
                    move = time.time() - start
                    pf.move(v, w * 0.9, ts)
                else:
                    robot.move(0,0)
                    pf.move(0,0,0)

                # print("Particle Move:" + str(move))



                start = time.time()
                # pf.show(1, 'Move', save_figure=True)
                show = time.time() - start
                # print("Show" + str(show))

                if overflow:
                    print('Loop time: {0:.3f} s'.format(loop_time))

                if localized:

                    coord = clustering.coordinates()
                    found = 1

                    start_point = (coord[0],coord[1])
                    start_angle = coord[2]
                    print("I'M AT ANGLE (deg):" + str(start_angle * 180 / 3.14))
                    # print(start_point)

                    robot.move(0, 0)
                    goal = (4, 4)
                    action_costs = (1.0, 2.0, 2.0, 2.0)  # Straight, Back, Turn Left, Turn Right
                    planning = Planning(m, action_costs, naive=False)
                    path = planning.a_star(start_point, goal)
                    smoothed_path = planning.smooth_path(path, data_weight=0.8, smooth_weight=0.1)
                    del path[0]
                    del smoothed_path[0]
                    path.insert(0, start_point)
                    smoothed_path.insert(0, start_point)
                    # planning.show(path, smoothed_path, blocking=True)
                    print(path)
                    count = 0
                    angle_old = 0
                    print('The robot is localized in point ' + str(coord))
                else:
                    print('The robot is lost.')

                # if count == 550:
                #     found = 1
                #     points = []
                #     for i in range (0,len(pf._particles)):
                #         a = Point(pf._particles[i])
                #         points.append(a)
                #     clusters, iterations = kmeans(points, 3, 0.3)
                #
                #
                #     num = 0
                #     for c in clusters:
                #         print(c.centroid.coords)
                #         print(len(c.points))
                #         if len(c.points) > num:
                #             sum_angles = 0
                #             num = len(c.points)
                #             for point in c.points:
                #                 sum_angles += point.coords[2]
                #             centroid = c.centroid
                #             start_angle = sum_angles/num
                #
                #
                #
                #     start_point = (centroid.coords[0],centroid.coords[1])
                #     print("I'M AT ANGLE (deg):" + str(start_angle * 180 / 3.14))
                #     # print(start_point)
                #
                #     robot.move(0, 0)
                #     goal = (4, 4)
                #     action_costs = (1.0, 100.0, 1.0, 1.0)  # Straight, Back, Turn Left, Turn Right
                #     planning = Planning(m, action_costs, naive=False)
                #     path = planning.a_star(start_point, goal)
                #     smoothed_path = planning.smooth_path(path, data_weight=0.8, smooth_weight=0.1)
                #     del path[0]
                #     del smoothed_path[0]
                #     path.insert(0,start_point)
                #     smoothed_path.insert(0, start_point)
                #     planning.show(path, smoothed_path, blocking=True)
                #     print(path)
                #     count = 0
                #     angle_old = 0

                count += 1
                control = 0

            while found == 1:
                loop_time, overflow = idle.task()
                if counting % (time_giros + time_recta) == 0:
                    if counter_path != len(path):
                        robot.move(0,0)
                        angle = math.atan2((path[counter_path + 1][1] - path[counter_path][1]),
                                           (path[counter_path + 1][0] - path[counter_path][0]))
                        print("I'm at point" + str(path[counter_path]))
                        # print("ANGLE:" + str(angle))
                        # print("OLD ANGLE:" + str(angle_old))
                        if counter_path == 0:
                            angle_turn = (angle - start_angle)
                            if angle_turn > 4:
                                angle_turn = 2 * 3.14 - angle_turn
                            if angle_turn < -4:
                                angle_turn = -2 * 3.14 + angle_turn
                            w = angle_turn / (time_giros * ts)
                            print("ANGLE TURN:" + str(angle - start_angle))
                        else:
                            angle_turn = (angle - angle_old)
                            if angle_turn > 4:
                                angle_turn = 2 * 3.14 - angle_turn
                            if angle_turn < -4:
                                angle_turn = -2 * 3.14 + angle_turn
                            w = angle_turn / (time_giros * ts) * 0.9
                            print("ANGLE TURN:" + str(angle - angle_old))
                        distance = math.sqrt((path[counter_path + 1][1] - path[counter_path][1]) ** 2 + (
                                path[counter_path + 1][0] - path[counter_path][0]) ** 2)
                        v = distance / (time_recta * ts)
                        print("DISTANCIA:" + str(distance))
                        # print(path[counter_path])
                        counter_path += 1
                        angle_old = angle
                elif counting % (time_giros + time_recta) <= time_giros:
                    if angle_turn != 0:
                        robot.move(0,w)
                    else:
                        counting += 4
                elif counting % (time_giros + time_recta) > time_giros and counting > time_giros:
                    z = robot.sense()
                    _, w = navigation.explore2(z, error_acumulation)
                    if ((z[4 - 1] - 1) + (z[5 - 1] - 1)) < -1.3:
                        robot.move(0,0)
                    else:
                        if w > 0.1:
                            robot.move(v * 1.25, w)
                        else:
                            robot.move(v * 1.1, w)
                counting += 1


    except KeyboardInterrupt:  # Break the infinite loop to gracefully close the connection
        pass

    # Close the connection
    vrep.simxGetPingTime(client_id)  # Make sure that the last command sent out had time to arrive
    vrep.simxFinish(client_id)
