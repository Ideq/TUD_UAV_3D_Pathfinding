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
from pylab import *




#start Connection to V-REP
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

##generate mapdata, load data if mapdata for scene exist
mapdata=UAV_mapgen.mapgen_fast("hexagon_new",44,44,20,clientID)
##mapdata=UAV_mapgen.mapgen("testroom111",12,12,4,clientID)
#
##start the synchron-simulation-mode to improve the communication between python and V-REP, while the simulation is running
#vrep.simxSynchronous(clientID,True); 
#
##Get goal-data from V-REP
goal_position=UAV_VREP.getPosition(clientID,'goal_new')
#print goal_position
#   
#
##Get start-data from v-REP
start_position=UAV_VREP.getPosition(clientID,'UAV')
#print start_position
#             
##Start pathfinding
#print "start pathfinding"
#
path=UAV_pathfinding.search(goal_position,start_position,"astar",3,mapdata)
#print "pathfinding finished"
#
##function to start the signals to transport the data to V-REP(LUA) and give the signal to the UAV-script, that the path is ready
UAV_VREP.show_path(path,clientID)
#
##function to follow the path, generates the needed velocities and heights, that are needed for the LUA-script and streams the needed signals
xyzarray,xyzneararray=UAV_VREP.followPath(clientID,path,goal_position)
