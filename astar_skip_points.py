# -*- coding: utf-8 -*-
"""
Created on Tue Jan 19 01:39:50 2016

@author: lijinke
"""
#from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as pl
from itertools import product, combinations
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

figure = plt.figure(figsize=(6,4), facecolor='w')
ax = figure.gca(projection='3d')

r = [1.2,2.8]
r_z = [0,1.2]
for s, e in combinations(np.array(list(product(r,r,r_z))), 2):
    if np.sum(np.abs(s-e)) == r[1]-r[0]:
        ax.plot3D(*zip(s,e), color="r")
    if np.sum(np.abs(s-e)) == r_z[1]-r_z[0]:
        ax.plot3D(*zip(s,e), color="r")
        

ax.plot(astar_x_3, astar_y_3, astar_z_3, c='b', zorder = 0.5)
ax.scatter(astar_x_3, astar_y_3, astar_z_3, zdir='z', s=60, c='b',marker='o',depthshade=False, zorder = 0.3) 
ax.scatter(astar_x_2, astar_y_2, astar_z_2, zdir='z', s=20, c='y',marker='o',depthshade=False, zorder = 0.1) 

ax.set_xlabel('x in m',fontsize = 20)
ax.set_ylabel('y in m',fontsize = 20)
ax.set_zlabel('z in m',fontsize = 20)
ax.xaxis._axinfo['label']['space_factor'] = 3
ax.yaxis._axinfo['label']['space_factor'] = 2.5
ax.zaxis._axinfo['label']['space_factor'] = 3
ax.set_xticks([0,1,2,3,4])
ax.set_yticks([0,1,2,3,4])
ax.set_zticks([0,1])

ax.tick_params(labelsize=15)
##plt.zlim(0,3)
blue=pl.lines.Line2D([0],[0],color="b",marker='o',markersize=8)
yellow=pl.lines.Line2D([0],[0],color="y",marker='o',linestyle='none',markersize=5)

ax.legend([blue,yellow], ['A* with skipped points', 'A* points'],fontsize = 20,loc=(0.1,0.8))

