# -*- coding: utf-8 -*-
"""
Created on Sun Nov 01 15:34:40 2015

@author: Jonas
"""

#UAV_main.py
import vrep
import UAV_mapgen
import UAV_pathfinding_astar
import UAV_VREP
import numpy as np
import time
import pathfollowing
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#from PathGenerationCubic import path_3d
#from FieldGeneration import calc_vec_field_fast
#from mayavi import mlab
#from pylab import array

#start Connection to V-REP
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
data=[0,0,10,0,0,0]
packedData=vrep.simxPackFloats(data)
vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
#generate mapdata, load data if mapdata for scene exist not implemented till now   
mapdata=UAV_mapgen.mapgen_fast("hexagon_new",44,44,20,clientID)
#mapdata=UAV_mapgen.mapgen("testroom111",12,12,4,clientID)
   

#Get goal-data from V-REP
goal_position=UAV_VREP.getPosition(clientID,'goal_new')
print goal_position
   

#Get start-data from v-REP
start_position=UAV_VREP.getPosition(clientID,'UAV_target')
print start_position

#part to secure more disitance to obstacles
#mapdata2=mapdata
#mapdata3=UAV_mapgen.save(mapdata2)
             
#Start pathfinding
print "start pathfinding"
start_time = time.time()
path=UAV_pathfinding_astar.search(goal_position,start_position,"astar",3,mapdata)
#path=UAV_pathfinding_astar.testpath()
#print path
    #function
        #input
            #goal_position, start_position, type of algorythm(A* or RRT), type of interpolation(1=linear, 2=quadratic, 3=qubic)
        #output
            #array, named path, contains all points of the smothed interpolated path

#Path visualation
    #function, creates an arrow an every point of the path pointing to the next point
        #input 
            #path, color(1=green,2=yewllow,3=red)
#UAV_VREP.showPath(clientID,path,1)
path_ready=1

UAV_VREP.show_path2(path,clientID)
#Path following
    #function
        #input
            #path

#UAV_VREP.followPath(clientID,path)
xyzarray,xyzneararray=UAV_VREP.followPath2(clientID,path,goal_position)
#start2=m_to_grid(start_position)
#goal2=m_to_grid(goal_position)
#Pos0 = array([4, 8, 3])
#Pos1 = array([14, 12, 17])
#Pos2 = array([14, 4, 17])
Loop = False
#Pos = array([Pos0, Pos1, Pos2])

# We obtain a path from a set of discrete points using
# Cubic splines

##X, Y, Z, Tt = path_3d(Pos, Loop)
#X, Y, Z, Tt = path_3d(np.transpose(path), Loop)
## Now we calculate the vector field using the provided functions
#
#vector_field_3D, Xc, Yc, Zc = calc_vec_field_fast(X, Y, Z, 30, 1)


# We plot
"""mlab.figure(size=(800, 600))
mlab.flow(Xc, Yc, Zc, vector_field_3D[0], vector_field_3D[1], vector_field_3D[2], linetype='tube', seedtype='plane')
mlab.pipeline.vector_field(Xc, Yc, Zc, vector_field_3D[0], vector_field_3D[1], vector_field_3D[2], name='vector field')
mlab.points3d(X, Y, Z, Tt)
mlab.quiver3d(Xc, Yc, Zc, vector_field_3D[0], vector_field_3D[1], vector_field_3D[2])
mlab.show()
"""
#position=[1,2,3]
#v_result=findnearst(position, path)


xerror=abs(xyzarray[0]-xyzneararray[0])
yerror=abs(xyzarray[1]-xyzneararray[1])
zerror=abs(xyzarray[2]-xyzneararray[2])


xspace=np.linspace(0, np.size(xerror)-1, num=np.size(xerror))
yspace=np.linspace(0, np.size(yerror)-1, num=np.size(yerror))
zspace=np.linspace(0, np.size(zerror)-1, num=np.size(zerror))


#plt.plot(xspace,xerror,'r',yspace,yerror,'g',zspace,zerror,'b')
#plt.plot(xspace,np.sqrt(xerror*xerror+yerror*yerror))

#plt.plot(yspace,yerror)
#plt.plot(xspace,xerror)

plt.plot(xyzarray[0],xyzarray[1], 'ro', path[0], path[1], 'g-')
#plt.plot(xpnear,ypnear, 'r--', xp, yp, 'g')
#ax = plt.axes(projection='3d')
#ax.plot(path[0],path[1], path[2], '-b')


