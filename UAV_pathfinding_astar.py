# -*- coding: utf-8 -*-
"""
Created on Fri Oct 30 00:17:18 2015

@author: renchen
"""

#import from Libaries which are usefull
import vrep#needed for the Connection with the Simulator
import sys
import numpy as np#needed for the arrays and some other mathematical operations
import time
import math
from scipy import interpolate#needed for the interpolation functions
import collections #needed for the queue       
import heapq#needed for the queue



#this queue structure is needed for the A* algorythm and the difference to the Dijkstra algorythm, which would return the same result, but normally needs more time
class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    
    
#this function is the difference between A* and Dijkstra, it returns the distance between a node and the goal, if 2 paths have the same cost it will use the path which is nearer to the goal       
def heuristic(a, b):
    (x1, y1, z1) = a
    (x2, y2, z2) = b
    return math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)

#this is the Pathfinding algorythm A*, implemented by using the defined functions and datastructures
def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            
            break
        for next in graph.neighbors(current):
            
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
                
    
    return came_from, cost_so_far
    
    





vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP


#create a grid
grid=SquareGrid(30,30,10)
#start the Pathfinding algorythm
came_from, cost_so_far = a_star_search(grid, (0, 0, 0), (16, 16, 0))

#this function is need to get the path(in points, nodes) from the results of the pathfinding algorythm
def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

#use the reconstruct function to get the path
path2=reconstruct_path(came_from,(0,0,0),(16,16,0))

#need to control the quadrocopter later, by just moving away the target slowly
errorCode,UAV=vrep.simxGetObjectHandle(clientID,'UAV_target',vrep.simx_opmode_oneshot_wait)

