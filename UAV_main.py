# -*- coding: utf-8 -*-
"""
Created on Sun Nov 01 15:34:40 2015

@author: Jonas
"""

#UAV_main.py
import vrep
import UAV_mapgen
import UAV_pathfinding
import UAV_VREP
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#start Connection to V-REP
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

#generate mapdata, load data if mapdata for scene exist not implemented till now   
mapdata=UAV_mapgen.mapgen_fast("hexagon_new2",44,44,20,clientID)
#mapdata=UAV_mapgen.mapgen("testroom111",12,12,4,clientID)

#start the synchron-simulation-mode to improve the communication between python and V-REP, while the simulation is running
vrep.simxSynchronous(clientID,True); 

#Get goal-data from V-REP
goal_position=UAV_VREP.getPosition(clientID,'goal_new')
print goal_position
   

#Get start-data from v-REP
start_position=UAV_VREP.getPosition(clientID,'UAV')
print start_position
             
#Start pathfinding
print "start pathfinding"
start_time = time.time()
path=UAV_pathfinding.search(goal_position,start_position,"astar",3,mapdata)
print "pathfinding finished"
    #function "UAV_pathfinding.search()"
        #input
            #goal_position, start_position, type of algorythm(A* or RRT), type of interpolation(1=linear, 2=quadratic, 3=qubic)
        #output
            #array, named path, contains all points of the smothed interpolated path

#function to start the signals to transport the data to V-REP(LUA) and give the signal to the UAV-script, that the path is ready
UAV_VREP.show_path(path,clientID)

#function to follow the path, generates the needed velocities and heights, that are needed for the LUA-script and streams the needed signals
xyzarray,xyzneararray=UAV_VREP.followPath(clientID,path,goal_position)

# Plot
xerror=abs(xyzarray[0]-xyzneararray[0])
yerror=abs(xyzarray[1]-xyzneararray[1])
zerror=abs(xyzarray[2]-xyzneararray[2])


xspace=np.linspace(0, np.size(xerror)-1, num=np.size(xerror))
yspace=np.linspace(0, np.size(yerror)-1, num=np.size(yerror))
zspace=np.linspace(0, np.size(zerror)-1, num=np.size(zerror))

#plot the path over the real movement in 2D(xy-plane)
plt.plot(xyzarray[0],xyzarray[1], 'ro', path[0], path[1], 'g-')

#somemore plot-suggestions that are possible showing different aspects of the path-following
#plt.plot(xspace,xerror,'r',yspace,yerror,'g',zspace,zerror,'b')
#plt.plot(xspace,np.sqrt(xerror*xerror+yerror*yerror))

#plt.plot(yspace,yerror)
#plt.plot(xspace,xerror)


#plt.plot(xpnear,ypnear, 'r--', xp, yp, 'g')
#ax = plt.axes(projection='3d')
#ax.plot(path[0],path[1], path[2], '-b')


