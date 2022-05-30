# %matplotlib qt
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D


def plot_tri_2(ax, x_, y_, z_, vertices_to_connect, alpha, color):
    for tr in vertices_to_connect:
        i,j,k = tr
        x = np.array([x_[i], x_[j], x_[k]])
        y = np.array([y_[i], y_[j], y_[k]])
        z = np.array([z_[i], z_[j], z_[k]])
        verts = [list(zip(x,y,z))]
        srf = Poly3DCollection(verts, alpha=alpha, facecolor="black", edgecolor="black")
        
        plt.gca().add_collection3d(srf)
        ax.add_collection3d(Poly3DCollection(verts))


    #ax.scatter(x_, y_, z_, color='b')


# np.random.seed(0)
# x = 2.0 * np.random.rand(20) - 1.0
# y = 2.0 * np.random.rand(20) - 1.0
# z = 2.0 * np.random.rand(20) - 1.0
# points = np.vstack([x, y, z]).T
# print(points)
# tri = Delaunay(points)

import sys
x_face = np.asarray(sys.argv[1].split(','), dtype=float)
y_face = np.asarray(sys.argv[2].split(','), dtype=float)
z_face = np.asarray(sys.argv[3].split(','), dtype=float)
vertices_to_connect_face = np.asarray(sys.argv[4].split(','), dtype=int)
vertices_to_connect_face = [vertices_to_connect_face[x:x+3] for x in range(0, len(vertices_to_connect_face), 3)]
x_front = np.asarray(sys.argv[5].split(','), dtype=float)
y_front = np.asarray(sys.argv[6].split(','), dtype=float)
z_front = np.asarray(sys.argv[7].split(','), dtype=float)
vertices_to_connect_front = np.asarray(sys.argv[8].split(','), dtype=int)
vertices_to_connect_front = [vertices_to_connect_front[x:x+3] for x in range(0, len(vertices_to_connect_front), 3)]
fig = plt.figure()
ax = Axes3D(fig)
fig.add_axes(ax)
plot_tri_2(ax, x_front,y_front,z_front, vertices_to_connect_front, 0.99, "g")
plot_tri_2(ax, x_face,y_face,z_face, vertices_to_connect_face, 0.9, "r")
plt.show(block=True)