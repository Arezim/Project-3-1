import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Sphere:

    def __init__(self, centre, radius=1.0, semi=False):
        self.centre = centre
        self.radius = radius
        self.coordinates_list = []
        n = 7
        k = 3
        if not semi:
            k *= 2

        rho = np.linspace(0, np.pi * 2, n)[:-1]
        if semi:
            gamma = np.linspace(0, np.pi / 2, k)[1:]
        else:
            gamma = np.linspace(0, np.pi, k)[1:-1]

        # lower point
        x = centre[0]
        y = centre[1]
        z = centre[2] + radius
        self.coordinates_list.append(np.array([x, y, z]))

        # pi
        if not semi:
            z = centre[2] - radius
            self.coordinates_list.append(np.array([x, y, z]))

        for a in gamma:
            x = centre[0] + radius * np.cos(rho) * np.sin(a)
            y = centre[1] + radius * np.sin(rho) * np.sin(a)
            z = centre[2] + radius * np.cos(a)
            for i in range(n-1):
                self.coordinates_list.append(np.array([x[i], y[i], z]))

        self.scores = []

    def average_scores(self):
        if len(self.scores) > 0:
            return np.mean(self.scores)
        else:
            return 0


if __name__ == '__main__':
    s = Sphere(centre=[0, 0, 0], radius=0.5, semi=False)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_ylim(-4, 4)
    ax.set_xlim(-4, 4)
    ax.set_zlim(-4, 4)
    ax.view_init(elev=0., azim=0)
    for p in s.coordinates_list:
        ax.scatter(p[0], p[1], p[2], c='b', alpha=0.25)
    plt.show()
