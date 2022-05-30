# %matplotlib qt
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial import Delaunay

def plot_tri_2(ax, x_, y_, z_, vertices_to_connect):
    x = np.array([])
    y = np.array([])
    z = np.array([])
    for tr in vertices_to_connect:
        i,j,k = tr
        x = np.append(x, [x_[i], x_[j], np.nan])      
        y = np.append(y, [y_[i], y_[j], np.nan])      
        z = np.append(z, [z_[i], z_[j], np.nan])
        x = np.append(x, [x_[j], x_[k], np.nan])      
        y = np.append(y, [y_[j], y_[k], np.nan])      
        z = np.append(z, [z_[j], z_[k], np.nan])
        x = np.append(x, [x_[i], x_[k], np.nan])      
        y = np.append(y, [y_[i], y_[k], np.nan])      
        z = np.append(z, [z_[i], z_[k], np.nan])
    ax.plot3D(x, y, z, color='g', lw='0.5')

    #ax.scatter(x_, y_, z_, color='b')


# np.random.seed(0)
# x = 2.0 * np.random.rand(20) - 1.0
# y = 2.0 * np.random.rand(20) - 1.0
# z = 2.0 * np.random.rand(20) - 1.0
# points = np.vstack([x, y, z]).T
# print(points)
# tri = Delaunay(points)

import sys
x = np.asarray(sys.argv[1].split(','), dtype=float)
y = np.asarray(sys.argv[2].split(','), dtype=float)
z = np.asarray(sys.argv[3].split(','), dtype=float)
vertices_to_connect = np.asarray(sys.argv[4].split(','), dtype=int)
vertices_to_connect = [vertices_to_connect[x:x+3] for x in range(0, len(vertices_to_connect), 3)]
fig = plt.figure()
ax = plt.axes(projection='3d')
plot_tri_2(ax, x,y,z, vertices_to_connect)
plt.show(block=True)