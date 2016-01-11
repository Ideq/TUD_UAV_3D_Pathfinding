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
data=[0,0,1,0,0,0]
packedData=vrep.simxPackFloats(data)
vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)

#generate mapdata, load data if mapdata for scene exist not implemented till now   
mapdata=UAV_mapgen.mapgen_fast("hexagon",32,32,10,clientID)
#mapdata=UAV_mapgen.mapgen("testroom111",12,12,4,clientID)
   

#Get goal-data from V-REP
goal_position=UAV_VREP.getPosition(clientID,'goal_new')
print goal_position
   

#Get start-data from v-REP
start_position=UAV_VREP.getPosition(clientID,'UAV_target')
print start_position

             
#Start pathfinding
print "start pathfinding"
start_time = time.time()
path=UAV_pathfinding_astar.search(goal_position,start_position,"astar",3,mapdata)

print("--- %s seconds ---" % (time.time() - start_time))
#print path
    #function
        #input
            #goal_position, start_position, type of algorythm(A* or RRT), type of interpolation(1=linear, 2=quadratic, 3=qubic)
        #output
            #array, named path, contains all points of the smothed interpolated path


#function to start the signals to transport the data to V-REP(LUA) and give the signal to the UAV-script, that the path is ready
UAV_VREP.show_path2(path,clientID)


#function to follow the path, generates the needed velocities and heights, that are needed for the LUA-script and streams the needed signals
UAV_VREP.followPath2(clientID,path,goal_position)



