# -*- coding: utf-8 -*-
"""
Created on Thu Oct 22 18:47:03 2015

@author: Jonas
"""

import vrep
import sys
import numpy as np
import time
import math
from scipy import interpolate

class SquareGrid:
    def __init__(self, xmax, ymax, zmax):
        self.xmax = xmax
        self.ymax = ymax
        self.zmax = zmax
    
    def cost(self, a, b):
        (x1, y1, z1) = a
        (x2, y2, z2) = b
        return math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)
        #return 1
        
    def in_bounds(self, id):
        (x, y, z) = id
        return 0 <= x < self.xmax and 0 <= y < self.ymax and 0 <= z < self.zmax
    
    def passable(self, id):
        (x,y,z)=id
        if arr[x,y,z]==0:
            boo=3
        else:
            boo=2
        return boo==3
    
    def neighbors(self, id):
        (x, y, z) = id
        results = [(x+1, y, z), (x, y-1, z), (x-1, y, z), (x, y+1, z),(x+1,y+1, z),(x+1,y-1, z),(x-1,y-1, z),(x-1,y+1, z),
                   (x, y, z+1),(x+1, y, z+1), (x, y-1, z+1), (x-1, y, z+1), (x, y+1, z+1),(x+1,y+1, z+1),(x+1,y-1, z+1),(x-1,y-1, z+1),(x-1,y+1, z+1),
                   (x, y, z-1),(x+1, y, z-1), (x, y-1, z-1), (x-1, y, z-1), (x, y+1, z-1),(x+1,y+1, z-1),(x+1,y-1, z-1),(x-1,y-1, z-1),(x-1,y+1, z-1)]
        #if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results
        
import collections        
import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    
    
        
def heuristic(a, b):
    (x1, y1, z1) = a
    (x2, y2, z2) = b
    #return abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2)
    return math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)
    #return 1

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

#arr=np.ndarray(shape=(30,30,10),dtype=float)
#
#errorCode,sensor1=vrep.simxGetObjectHandle(clientID,'Sensor_1',vrep.simx_opmode_oneshot_wait)
#errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor1,vrep.simx_opmode_streaming)            
#
#errorCode,sensor2=vrep.simxGetObjectHandle(clientID,'Sensor_2',vrep.simx_opmode_oneshot_wait)
#errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor2,vrep.simx_opmode_streaming)            
#
#time.sleep(3)
#                                  
#
#index=0
#index2=0
#index3=0
#for index in range(30):
#    for index2 in range(30):
#        for index3 in range(10):
#        #for index3 in range(1):
#             #Block an neue Position
#            x=0.4+0.4*index
#            y=0.2+0.4*index2
#            z=0.3+0.4*index3
#            vrep.simxSetObjectPosition (clientID,sensor1,-1,(x,y,z),vrep.simx_opmode_oneshot)
#            vrep.simxSetObjectPosition (clientID,sensor2,-1,(x-0.4,y,z),vrep.simx_opmode_oneshot)
#            time.sleep(0.2)            
#            #Kollision prüfen 
#            #vrep.simxGetCollisionHandle (regular API equivalent: simGetCollisionHandle)
#            errorCode,detectionState1,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor1,vrep.simx_opmode_buffer)            
#            errorCode,detectionState2,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor2,vrep.simx_opmode_buffer)            
#                        
#            if (detectionState1 or detectionState2):            
#                arr[index,index2,index3]=1   #Ergebnis speichern
#            else:
#                arr[index,index2,index3]=0 
#            #time.sleep(0.2)


grid=SquareGrid(30,30,10)
#Kosten=grid.cost((1,1),(5,3))
came_from, cost_so_far = a_star_search(grid, (0, 0, 0), (8, 16, 0))
def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

path2=reconstruct_path(came_from,(0,0,0),(8,16,0))
path3=path2
errorCode,UAV=vrep.simxGetObjectHandle(clientID,'UAV_target',vrep.simx_opmode_oneshot_wait)


def collision(a,b):
    (x1,y1,z1)=a
    (x2,y2,z2)=b
    out=0;
    #Gerade zur Interpolation zwischen 2 Punkten
    for l in range(1000):
        
        x=x1+(x2-x1)*(l+1)/1000      
        
        y=y1+(y2-y1)*(l+1)/1000      
        
        z=z1+(z2-z1)*(l+1)/1000       
        
        x=round(x,0)
        y=round(y,0)
        z=round(z,0)
        
        out=out+arr[x,y,z]
    return out==0

def winkel_berechnen1(a,b):
    dot = np.dot(a,b)
    x_modulus = np.sqrt(a[0]**2+a[1]**2+a[2]**2)
    y_modulus = np.sqrt(b[0]**2+b[1]**2+b[2]**2)
    cos_angle = dot / x_modulus / y_modulus 
    angle = np.arccos(cos_angle) # Winkel in Bogenmaß 
    ang2=angle*360/2/np.pi
    #print ang2
    if a[1]>0:
        return angle
    else:
        return -angle

def winkel_berechnen2(a,b):
    dot = np.dot(a,b)
    x_modulus = np.sqrt(a[0]**2+a[1]**2+a[2]**2)
    y_modulus = np.sqrt(b[0]**2+b[1]**2+b[2]**2)
    cos_angle = dot / x_modulus / y_modulus 
    angle = np.arccos(cos_angle) # Winkel in Bogenmaß 
    ang2=angle*360/2/np.pi
    #print ang2
    if a[2]>0:
        return angle
    else:
        return -angle
       
#interpolation
#1. Schritt unnötige Zwischenknoten eliminieren
weiter=1
while weiter>0:
    weiter=0
    i=0
    while i <(len(path2)-2):
        if collision(path2[i],path2[i+2]):
            path2.pop(i+1)
            weiter=1
        i=i+1
#2. Schritt
data=np.ndarray(shape=(len(path2),3),dtype=float)
for i in range(len(path2)):
    (x,y,z)=path2[i]
    data[i,0]=x
    data[i,1]=y
    data[i,2]=z

data = data.transpose()
tck, u= interpolate.splprep(data,k=1)
new1 = interpolate.splev(np.linspace(0,1,100), tck)
tck, u= interpolate.splprep(data,k=2)
new2 = interpolate.splev(np.linspace(0,1,100), tck)
tck, u= interpolate.splprep(data,k=3)
new3 = interpolate.splev(np.linspace(0,1,100), tck)

errorCode,Arrow=vrep.simxGetObjectHandle(clientID,'Arrow',vrep.simx_opmode_oneshot_wait)
objectHandles=np.array([Arrow])
#trajektorie abfliegen, indem das Ziel immer weiter geschoben wird
for next in range(100):
    a1=new1[0]
    b1=new1[1]
    c1=new1[2]
    a=a1[next]
    b=b1[next]
    c=c1[next]
    if next<99:
        a2=a1[next+1]
        b2=b1[next+1]
        c2=c1[next+1]
    else:
        a2=a
        b2=b
        c2=c
    adiff=a2-a
    bdiff=b2-b
    cdiff=c2-c
    x=0.4+0.4*a
    y=0.2+0.4*b
    z=0.3+0.4*c
    returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
    Arro=newObjectHandles[0]
    vrep.simxSetObjectPosition (clientID,Arro,-1,(0,0,0),vrep.simx_opmode_oneshot)
    returnCode=vrep.simxSetObjectOrientation(clientID,Arro,-1,[winkel_berechnen1([0,bdiff,cdiff],[0,0,-1]),-winkel_berechnen2([0,bdiff,cdiff],[adiff,bdiff,cdiff]),0],vrep.simx_opmode_oneshot) #winkel_berechnen([0,bdiff,cdiff],[adiff,bdiff,cdiff])
    #returnCode=vrep.simxSetObjectOrientation(clientID,Arro,-1,[np.pi/2,0,0],vrep.simx_opmode_oneshot)    
    vrep.simxSetObjectPosition (clientID,Arro,-1,(x,y,z),vrep.simx_opmode_oneshot)

errorCode,Arrow=vrep.simxGetObjectHandle(clientID,'Arrow2',vrep.simx_opmode_oneshot_wait)
objectHandles=np.array([Arrow])
#trajektorie abfliegen, indem das Ziel immer weiter geschoben wird
for next in range(100):
    a1=new2[0]
    b1=new2[1]
    c1=new2[2]
    a=a1[next]
    b=b1[next]
    c=c1[next]
    if next<99:
        a2=a1[next+1]
        b2=b1[next+1]
        c2=c1[next+1]
    else:
        a2=a
        b2=b
        c2=c
    adiff=a2-a
    bdiff=b2-b
    cdiff=c2-c
    x=0.4+0.4*a
    y=0.2+0.4*b
    z=0.3+0.4*c
    returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
    Arro=newObjectHandles[0]
    vrep.simxSetObjectPosition (clientID,Arro,-1,(0,0,0),vrep.simx_opmode_oneshot)
    returnCode=vrep.simxSetObjectOrientation(clientID,Arro,-1,[winkel_berechnen1([0,bdiff,cdiff],[0,0,-1]),-winkel_berechnen2([0,bdiff,cdiff],[adiff,bdiff,cdiff]),0],vrep.simx_opmode_oneshot) #winkel_berechnen([0,bdiff,cdiff],[adiff,bdiff,cdiff])
    #returnCode=vrep.simxSetObjectOrientation(clientID,Arro,-1,[np.pi/2,0,0],vrep.simx_opmode_oneshot)    
    vrep.simxSetObjectPosition (clientID,Arro,-1,(x,y,z),vrep.simx_opmode_oneshot)

errorCode,Arrow=vrep.simxGetObjectHandle(clientID,'Arrow3',vrep.simx_opmode_oneshot_wait)
objectHandles=np.array([Arrow])
#trajektorie abfliegen, indem das Ziel immer weiter geschoben wird
for next in range(100):
    a1=new3[0]
    b1=new3[1]
    c1=new3[2]
    a=a1[next]
    b=b1[next]
    c=c1[next]
    if next<99:
        a2=a1[next+1]
        b2=b1[next+1]
        c2=c1[next+1]
    else:
        a2=a
        b2=b
        c2=c
    adiff=a2-a
    bdiff=b2-b
    cdiff=c2-c
    x=0.4+0.4*a
    y=0.2+0.4*b
    z=0.3+0.4*c
    returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
    Arro=newObjectHandles[0]
    vrep.simxSetObjectPosition (clientID,Arro,-1,(0,0,0),vrep.simx_opmode_oneshot)
    returnCode=vrep.simxSetObjectOrientation(clientID,Arro,-1,[winkel_berechnen1([0,bdiff,cdiff],[0,0,-1]),-winkel_berechnen2([0,bdiff,cdiff],[adiff,bdiff,cdiff]),0],vrep.simx_opmode_oneshot) #winkel_berechnen([0,bdiff,cdiff],[adiff,bdiff,cdiff])
    #returnCode=vrep.simxSetObjectOrientation(clientID,Arro,-1,[np.pi/2,0,0],vrep.simx_opmode_oneshot)    
    vrep.simxSetObjectPosition (clientID,Arro,-1,(x,y,z),vrep.simx_opmode_oneshot)    
    #vrep.simxSetObjectPosition (clientID,UAV,-1,(x,y,z),vrep.simx_opmode_oneshot)
    #time.sleep(1)
#returnCode,position=vrep.simxGetJointPosition(clientID,UAV,vrep.simx_opmode_buffer)

#returnCode=vrep.simxSetJointTargetPosition(clientID,UAV,1.0,vrep.simx_opmode_oneshot)

print 'fertig'
