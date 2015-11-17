# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 15:31:37 2015

@author: lijinke
"""
import vrep#needed for the Connection with the Simulator
import sys
import numpy as np#needed for the arrays and some other mathematical operations
import time
import math
from scipy import interpolate#needed for the interpolation functions
import collections #needed for the queue       
import heapq#needed for the queue


#vrep.simxFinish(-1)
#clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP


def getPosition(clientID,goal_new):
    errorcode,newgoal_handle=vrep.simxGetObjectHandle(clientID,goal_new,vrep.simx_opmode_oneshot_wait)
    time.sleep(1) 
    errorCode,newgoal_position=vrep.simxGetObjectPosition(clientID,newgoal_handle,-1,vrep.simx_opmode_streaming)
    time.sleep(1) 
    errorCode,newgoal_position=vrep.simxGetObjectPosition(clientID,newgoal_handle,-1,vrep.simx_opmode_buffer)
    return newgoal_position
    
#position=getPosition(clientID,'goal_new')

def angle_calculationx(a,b):
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

def angle_calculationy(a,b):
    dot = np.dot(a,b)
    x_modulus = np.sqrt(a[0]**2+a[1]**2+a[2]**2)
    y_modulus = np.sqrt(b[0]**2+b[1]**2+b[2]**2)
    cos_angle = dot / x_modulus / y_modulus 
    angle = np.arccos(cos_angle) # angle in radiant
    if a[2]>0:
        return angle
    else:
        return -angle

def showPath(clientID,qubic,color):
    #Cubic
    errorCode,Arrow=vrep.simxGetObjectHandle(clientID,'Arrow3',vrep.simx_opmode_oneshot_wait)
    objectHandles=np.array([Arrow])
    #put arrows on every point the trajectory pointing in the direction of the next point
    for next in range(100):
        a1=qubic[0]
        b1=qubic[1]
        c1=qubic[2]
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
        returnCode=vrep.simxSetObjectOrientation(clientID,Arro,-1,[angle_calculationx([0,bdiff,cdiff],[0,0,-1]),-angle_calculationy([0,bdiff,cdiff],[adiff,bdiff,cdiff]),0],vrep.simx_opmode_oneshot) #winkel_berechnen([0,bdiff,cdiff],[adiff,bdiff,cdiff])
        vrep.simxSetObjectPosition (clientID,Arro,-1,(x,y,z),vrep.simx_opmode_oneshot)  

def show_path2(path2,clientID):
    errorCode,Ball=vrep.simxGetObjectHandle(clientID,'A_star_points',vrep.simx_opmode_oneshot_wait)
    objectHandles=np.array([Ball])
    for next in path2:
        (a,b,c)=next
        x=0.4+0.4*a
        y=0.2+0.4*b
        z=0.3+0.4*c
        returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
        Ball_new=newObjectHandles[0]
        vrep.simxSetObjectPosition (clientID,Ball_new,-1,(x,y,z),vrep.simx_opmode_oneshot)
        
def followPath(clientID,path):
    errorCode,UAV=vrep.simxGetObjectHandle(clientID,'UAV_target',vrep.simx_opmode_oneshot_wait)
    for next in range(100):
        a=path[0]
        b=path[1]
        c=path[2]
        a2=a[next]
        b2=b[next]
        c2=c[next]
        x=round(0.4+0.4*a2,2)
        y=round(0.2+0.4*b2,2)
        z=round(0.3+0.4*c2,2)
        vrep.simxSetObjectPosition (clientID,UAV,-1,(x,y,z),vrep.simx_opmode_oneshot)
        time.sleep(0.3)