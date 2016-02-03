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

#figure = plt.figure(figsize=(6,4), facecolor='w')
#ax = figure.gca(projection='3d')


#ax.plot(path[0],path[1], path[2], 'b')
#ax.plot(xyzneararray[0],xyzneararray[1], xyzneararray[2], 'r')

#ax.plot(Dpath_astar[0],Dpath_astar[1], Dpath_astar[2], 'm')
#ax.plot(Dpath1[0],Dpath1[1], Dpath1[2], 'c')
#ax.plot(Dpath5[0],Dpath5[1], Dpath5[2], 'c--')

#ax.plot(Dpath3[0],Dpath3[1], Dpath3[2], 'k--')
#ax.plot(Dpath5[0],Dpath5[1], Dpath5[2], 'y--')

#ax.scatter([0.498],[0.31],[0.6],color="g",s=80)
#ax.scatter([2.56],[2.35],[0.95],color="r",s=80)

#r = [1.5,2.5]
#r_z = [0,1]
#for s, e in combinations(np.array(list(product(r,r,r_z))), 2):
#    if np.sum(np.abs(s-e)) == r[1]-r[0]:
#        ax.plot3D(*zip(s,e), color="r")
#    if np.sum(np.abs(s-e)) == r_z[1]-r_z[0]:
#        ax.plot3D(*zip(s,e), color="r")
#        
#r_sx =[1.2,2.8]
#r_sy =[1.2,2.8]
#r_sz =[0,1.2]
#
#
#r_sxn=np.array([0.00,0.00])
#r_syn=np.array([0.00,0.00])
#r_szn=np.array([0.00,0.00])
#
#for indexx in [0, 1,2,3]:
#    indexxn = indexx*0.4
#    r_sxn[0]=r_sx[0] + indexxn
#    r_sxn[1]=r_sx[0] + indexxn +0.4
#    print r_sxn
#    for indexy in [0, 1,2,3]:
#        indexyn = indexy*0.4
#        
#        r_syn[0]=r_sy[0] + indexyn
#        r_syn[1]=r_sy[0] + indexyn +0.4
#        for indexz in [0,1,2]:
#            indexzn = indexz*0.4
#            r_szn[0]=r_sz[0] + indexzn
#            r_szn[1]=r_sz[0] + indexzn +0.4
#            for s, e in combinations(np.array(list(product(r_sxn,r_syn,r_szn))), 2):
#                if np.sum(np.abs(s-e)) == r_szn[1]-r_szn[0]:
#                    red=ax.plot3D(*zip(s,e), color="r",linewidth=0.2,label='Line r')
#                if np.sum(np.abs(s-e)) == r_sxn[1]-r_sxn[0]:
#                    ax.plot3D(*zip(s,e), color="r",linewidth=0.2)
#                if np.sum(np.abs(s-e)) == r_syn[1]-r_syn[0]:
#                    ax.plot3D(*zip(s,e), color="r",linewidth=0.2)

#r_sx=[0.8,1.2]
#r_sy=[0.0,0.4]
#r_sz=[1.2,1.6]
#a,b=combinations(np.array(list(product(r_sx,r_sy,r_sz))),2)
#for s, e in combinations(np.array(list(product(r_sx,r_sy,r_sz))), 2):
#    print np.sum(np.abs(s-e))
#    if np.sum(np.abs(s-e)) == r_sx[1]-r_sx[0]:
#        print np.sum(np.abs(s-e))
#        ax.plot3D(*zip(s,e), color="r")
#    if np.sum(np.abs(s-e)) == r_sy[1]-r_sy[0]:
#        print np.sum(np.abs(s-e))
#        ax.plot3D(*zip(s,e), color="r")
#for i in range(len(astar_x)):
    
#ax.scatter(astar_x, astar_y, astar_z, zdir='z', s=30, c='g',marker='o')  
#ax.plot(x, y, z, c='b')
#ax.plot(real_path[0,0:], real_path[1,0:],real_path[2,0:], c='g')  
#ax.scatter(astar_x_2, astar_y_2, astar_z_2, zdir='z', s=30, c='y',marker='o',depthshade=False) 
#ax.plot(astar_x_3, astar_y_3, astar_z_3, c='b', zorder = 0.5)

 
#ax.scatter(astar_x_2, astar_y_2, astar_z_2, zdir='z', s=30, c='b',marker='o',depthshade=False, zorder = 0.3) 
#ax.plot(astar_1_x, astar_1_y, astar_1_z, c='y') 
#ax.plot(rrt_2_x, rrt_2_y, rrt_2_z, c='y')  
#for i in range(1003):
#    #time_data1p[i]=time_data1p[i]-time_data1[1]
#    real_velo[1,i]=real_velo[1,i]-300
real_x=real_path[0,0:]
real_y=real_path[1,0:]
real_z=real_path[2,0:]
abs_error=[]
for i in range(len(real_x)):
    dx=real_x[i]-x
    dy=real_y[i]-y
    dz=real_z[i]-z
         
    distance=np.sqrt(dx*dx+dy*dy+dz*dz)  #caculate distance between position and every points in the path
    
    minindex=np.argmin(distance)
    error_x=real_x[i]-x[minindex]
    error_y=real_y[i]-y[minindex]
    error_z=real_z[i]-z[minindex]
    abs_error.append(error_x**2+error_y**2+error_z**2)
plot_time=np.linspace(0, 27.43, num=1221)
plt.plot(plot_time,abs_error,'r')  
#plt.plot(real_velo[1,1:]/1000,soll_velo[2,1:],'r') 
plt.xlim(0,27.43)
#plt.ylim(0.57,0.75)
plt.xlabel('$t$ in s',fontsize = 20)
plt.ylabel('$|\Delta x|$ in m',fontsize = 20)
plt.xticks(fontsize = 15)
plt.yticks(fontsize = 15)
#plt.legend(['real value','control input'],fontsize = 20)
#ax.set_xlim(0,3.5)
#ax.set_ylim(0,3.5)
#ax.set_zlim(0,1.2)
##
#ax.set_xlabel('x in m',fontsize = 20)
#ax.set_ylabel('y in m',fontsize = 20)
#ax.set_zlabel('z in m',fontsize = 20)
#ax.xaxis._axinfo['label']['space_factor'] = 2.5
#ax.yaxis._axinfo['label']['space_factor'] = 2.5
#ax.zaxis._axinfo['label']['space_factor'] = 2
#ax.set_xticks([0,1,2,3])
#ax.set_yticks([0,1,2,3])
#ax.set_zticks([0,1])
#
#ax.tick_params(labelsize=15)
##plt.zlim(0,3)
#blue=pl.lines.Line2D([0],[0],color="b",linewidth=2)
#red=pl.lines.Line2D([0],[0],color="g",linewidth=2)
#
##red=pl.lines.Line2D([0],[0],color="r",linewidth=1)
##plt.xlabel('$x_1$ in m',fontsize = 20)
##plt.ylabel('$x_2$ in m',fontsize = 20)
##plt.zlabel('$x_3$ in m',fontsize = 20)
#ax.legend([blue,red], ['calculated path','UAV movement'],fontsize = 20,loc=(0.1,0.8))

#bigger font like double size
#plt.xticks(fontsize = 15)
#plt.yticks(fontsize = 15)
#plt.legend(['A*','RRT','RRT'])
#ax.plot(path1[0],path1[1], path1[2], 'b')
#ax.plot(path3[0],path3[1], path3[2], 'b--')
#ax.plot(path4[0],path4[1], path4[2], 'r')


#plot1, =plt.plot(path1[0],path1[1], 'r',label='Line 1')
#plot2, =plt.plot(path3[0],path3[1], 'r--',label='Line 2')
#plot3, =plt.plot(path4[0],path4[1], 'b',label='Line 3')
#plt.legend([plot1, plot2, plot3], ['RRT', 'RRT', 'AStar'])
#plt.xlabel('x axis in m')
#plt.ylabel('y axis in m')
#plt.title('path')
