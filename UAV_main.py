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
#start Connection to V-REP
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
UAV_mapgen.mapgen_fast("testroom",12,12,4,clientID)
#generate mapdata, load data if mapdata for scene exist not implemented till now   
#mapdata=UAV_mapgen.mapgen("testroom",12,12,4,clientID)
    #function
        #input
            #scene-name,x(in m),y(in m),z(in m) 
        #output
            #array, named "mapdata"

#Get goal-data from V-REP
#goal_position=UAV_VREP.getPosition(clientID,'goal_new')
#goal_position=np.array([0,0,2])
    #function
        #output
            #(x,y,z), named goal_position
        #get the position of the goal object from V-REP

#Get start-data from v-REP
#start_position=UAV_VREP.getPosition(clientID,'UAV_target')
    #function
        #output
            #(x,y,z), named start_position
        #get the position of the quadrocopter from V-REP

#mapdata2=mapdata

#mapdata3=UAV_mapgen.save(mapdata2)
             
#Start pathfinding
#start_time = time.time()
#path=UAV_pathfinding_astar.search(goal_position,start_position,"astar",3,mapdata3)
#print("--- %s seconds ---" % (time.time() - start_time))
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

#Path following
    #function
        #input
            #path
#errorCode,UAV=vrep.simxGetObjectHandle(clientID,'UAV_target',vrep.simx_opmode_oneshot_wait)
#for next in range(100):
#    a=path[0]
#    b=path[1]
#    c=path[2]
#    a2=a[next]
#    b2=b[next]
#    c2=c[next]
#    x=round(0.4+0.4*a2,2)
#    y=round(0.2+0.4*b2,2)
#    z=round(0.3+0.4*c2,2)
#    vrep.simxSetObjectPosition (clientID,UAV,-1,(x,y,z),vrep.simx_opmode_oneshot)
#    time.sleep(0.3)