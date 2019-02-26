
import time
import os
from scipy.cluster.hierarchy import dendrogram, linkage, fcluster
from matplotlib import pyplot as plt

from robot_p3dx import RobotP3DX
from map import Map
from particle_filter import ParticleFilter


class Clustering:

    def __init__(self, pf_object: ParticleFilter):
        self._pf = pf_object
        self._Z = linkage(pf_object._particles)  # linkage matrix
        self._max_d = 3
        self._clusters = []
        self._elements = []

    def localize(self, max_d):
        self._Z = linkage(self._pf._particles)  # linkage matrix
        self._max_d = max_d  # max_d as in max_distance
        self._clusters = fcluster(self._Z, max_d, criterion='distance')  #clusters taking into account distance

        cluster_count = max(self._clusters)
        elements = []

        for i in range(1, cluster_count + 1):
            elements.append(len([j for j in self._clusters if i == j]))

        self._elements = elements
        print('Number of clusters: ' + str(cluster_count))
        print('Content of clusters: ' + str(elements))

        elements_max = max(elements)
        print('The biggest cluster has ' + str(elements_max) + ' elements.')

        if elements_max > len(self._pf._particles)/1.15:
            return True
        else:
            return False

    def show(self):
        plt.clf()

        fig = plt.figure(1)
        plt.title('Dendrogram (Iteration #' + str(pf._iteration) + ')')
        plt.xlabel('sample index')
        plt.ylabel('distance')
        dendrogram(
            self._Z,
            truncate_mode='none',  # show only the last p merged clusters
            p=5,  # show only the last p merged clusters
            leaf_rotation=90.,
            leaf_font_size=12.,
            show_contracted=True,  # to get a distribution impression in truncated branches

        )
        plt.axhline(y=self._max_d, c='k')

        plt.ion()  # Activate interactive mode
        plt.show()
        plt.pause(0.0001)  # Wait for 1 ms or the figure wont be displayed

        file_name = str(self.pf._iteration).zfill(4) + ' ' + 'dendrogram' + '.png'
        file_path = os.path.join('img', file_name)
        fig.savefig(file_path, box_inches='tight')

    def coordinates(self):
        max = 0
        index = 0
        clusters = self._clusters
        main_cluster = [0, 0, 0]
        for i in range(0, len(self._elements)):
            if self._elements[i] > max:
                max = self._elements[i]
                index = i + 1

        j = 0
        for particle in self._pf._particles:
            if clusters[j] == index:
                main_cluster[0] = main_cluster[0] + particle[0]
                main_cluster[1] = main_cluster[1] + particle[1]
                main_cluster[2] = main_cluster[2] + particle[2]
            j = j + 1

        main_cluster[:] = [x / self._elements[index - 1] for x in main_cluster]

        return tuple(main_cluster)


if __name__ == '__main__':

    m = Map('map_pf.json')
    pf = ParticleFilter(m, RobotP3DX.SENSORS[:8], RobotP3DX.SENSOR_RANGE, particle_count=500)
    clustering = Clustering(pf)
    dt = 1  # Sampling time [s]

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

    for u, z in zip(motions, measurements):
        # Solve differential kinematics
        v = (u[0] + u[1]) * RobotP3DX.WHEEL_RADIUS / 2
        w = (u[1] - u[0]) * RobotP3DX.WHEEL_RADIUS / RobotP3DX.TRACK

        # Move
        start = time.time()
        pf.move(v, w, dt)
        move = time.time() - start

        localized = clustering.localize(0.4)
        coord = (0, 0, 0)
        if localized:

            coord = clustering.coordinates()
            print('The robot is localized in point ' + str(coord))
        else:
            print('The robot is lost.')

        pf.show(1, 'Move', save_figure=True, centroid=coord)

        # Sense
        start = time.time()
        pf.resample(z)
        sense = time.time() - start

