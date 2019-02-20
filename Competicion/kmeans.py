import sys
import math
import random

from _functools import reduce

"""
Point clustering using Python [http://moderndata.plot.ly/point-clustering-in-python/]
This is an improved version of the KMeans Clustering Algorithmn
Which was originally implemented by 'Ian Danforth'.
[Reference: https://gist.github.com/iandanforth/5862470]
To properly use the Plotly integration:
1. Get a user credentials from "https://plot.ly/settings/api"
2. Install Plotly: [sudo] pip install plotly
"""


# a list containing points in each cluster per iteration
iteration_points = []

# a list containing the centroids in each cluster per iteration
iteration_centroids = []


def export_clusters(iterations, num_clusters):

    pass

def main():

    # points in the dataset
    num_points = 30

    # dimensions for each point
    dimensions = 2

    # range for the values of the points
    lower = 0
    upper = 300

    # number of clusters (K)
    num_clusters = 3

    # when optimization has 'converged'
    opt_cutoff = 0.5

    # generate some points
    points = [makeRandomPoint(dimensions, lower, upper) for i in xrange(num_points)]

    # cluster those data points
    clusters, iterations = kmeans(points, num_clusters, opt_cutoff)



class Point:
    '''
    An point in n dimensional space
    '''

    def __init__(self, coords):
        '''
        coords - A list of values, one per dimension
        '''

        self.coords = coords
        self.n = len(coords)

    def __repr__(self):
        return str(self.coords)


class Cluster:
    '''
    A set of points and their centroid
    '''

    def __init__(self, points):
        '''
        points - A list of point objects
        '''

        if len(points) == 0: raise Exception("ILLEGAL: empty cluster")
        # The points that belong to this cluster
        self.points = points

        # The dimensionality of the points in this cluster
        self.n = points[0].n

        # Assert that all points are of the same dimensionality
        for p in points:
            if p.n != self.n: raise Exception("ILLEGAL: wrong dimensions")

        # Set up the initial centroid (this is usually based off one point)
        self.centroid = self.calculateCentroid()

    def __repr__(self):
        '''
        String representation of this object
        '''
        return str(self.points)

    def update(self, points):
        '''
        Returns the distance between the previous centroid and the new after
        recalculating and storing the new centroid.
        '''
        old_centroid = self.centroid
        self.points = points
        self.centroid = self.calculateCentroid()
        shift = getDistance(old_centroid, self.centroid)
        return shift

    def calculateCentroid(self):
        '''
        Finds a virtual center point for a group of n-dimensional points
        '''
        numPoints = len(self.points)
        # Get a list of all coordinates in this cluster
        coords = [p.coords for p in self.points]
        # Reformat that so all x's are together, all y'z etc.
        unzipped = zip(*coords)
        # Calculate the mean for each dimension
        centroid_coords = [math.fsum(dList) / numPoints for dList in unzipped]

        return Point(centroid_coords)


def kmeans(points, k, cutoff):
    # Pick out k random points to use as our initial centroids
    initial = random.sample(points, k)

    # Create k clusters using those centroids
    clusters = [Cluster([p]) for p in initial]

    # Loop through the dataset until the clusters stabilize
    loopCounter = 0
    while True:
        # Create a list of lists to hold the points in each cluster
        lists = [[] for c in clusters]
        clusterCount = len(clusters)

        # Start counting loops
        loopCounter += 1

        # For every point in the dataset ...
        for p in points:
            # Get the distance between that point and the centroid of the first
            # cluster.
            smallest_distance = getDistance(p, clusters[0].centroid)

            # Set the cluster this point belongs to
            clusterIndex = 0

            # For the remainder of the clusters ...
            for i in range(clusterCount - 1):
                # calculate the distance of that point to each other cluster's
                # centroid.
                distance = getDistance(p, clusters[i + 1].centroid)
                # If it's closer to that cluster's centroid update what we
                # think the smallest distance is, and set the point to belong
                # to that cluster
                if distance < smallest_distance:
                    smallest_distance = distance
                    clusterIndex = i + 1
            lists[clusterIndex].append(p)

        # Set our biggest_shift to zero for this iteration
        biggest_shift = 0.0

        # store the iteration's cluster points and respective centroids
        # the first iteration will only have centroids in the cluster
        if loopCounter >= 2:
            global iteration_points
            global iteration_centroids

            iter_point_data = []
            iter_centroid_data = []

            for j in range(k):
                x, y = [], []

                for p in clusters[j].points:
                    x.append(p.coords[0])
                    y.append(p.coords[1])

                c = clusters[j].calculateCentroid()

                iter_point_data.append([x, y])
                iter_centroid_data.append([c.coords[0], c.coords[1]])

            iteration_points.append(iter_point_data)
            iteration_centroids.append(iter_centroid_data)

        # As many times as there are clusters ...
        for i in range(clusterCount):
            # Calculate how far the centroid moved in this iteration
            shift = clusters[i].update(lists[i])
            # Keep track of the largest move from all cluster centroid updates
            biggest_shift = max(biggest_shift, shift)

        # If the centroids have stopped moving much, say we're done!
        if biggest_shift < cutoff:
            print
            "Converged after %d iterations" % (loopCounter - 1)
            break

    return clusters, loopCounter - 1


def getDistance(a, b):
    '''
    Euclidean distance between two n-dimensional points.
    Note: This can be very slow and does not scale well
    '''
    if a.n != b.n:
        raise Exception("ILLEGAL: non comparable points")

    ret = reduce(lambda x, y: x + pow((a.coords[y] - b.coords[y]), 2), range(a.n), 0.0)
    return math.sqrt(ret)


def makeRandomPoint(n, lower, upper):
    '''
    Returns a Point object with n dimensions and values between lower and
    upper in each of those dimensions
    '''
    p = Point([random.uniform(lower, upper) for i in range(n)])
    return p


if __name__ == "__main__":
    main()