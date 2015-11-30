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
import pathfollowing
import UAV_pathfinding_astar

#vrep.simxFinish(-1)
#clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP


def getPosition(clientID,goal_new):
    errorcode,newgoal_handle=vrep.simxGetObjectHandle(clientID,goal_new,vrep.simx_opmode_oneshot_wait)
    #time.sleep(1) 
    errorCode,newgoal_position=vrep.simxGetObjectPosition(clientID,newgoal_handle,-1,vrep.simx_opmode_streaming)
    time.sleep(0.1) 
    errorCode,newgoal_position=vrep.simxGetObjectPosition(clientID,newgoal_handle,-1,vrep.simx_opmode_buffer)
    return newgoal_position
    
#position=getPosition(clientID,'goal_new')

def angle_calculationx(a,b):
    dot = np.dot(a,b)
    x_modulus = np.sqrt(a[0]**2+a[1]**2+a[2]**2)
    y_modulus = np.sqrt(b[0]**2+b[1]**2+b[2]**2)
    cos_angle = dot / x_modulus / y_modulus 
    angle = np.arccos(cos_angle) # Winkel in Bogenmaß 
    #ang2=angle*360/2/np.pi
    if b[1]>0:
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
    xpath=qubic[0]
    ypath=qubic[1]
    zpath=qubic[2]
    #print xpath
    for next in range(len(xpath)):
        x=xpath[next]
        y=ypath[next]
        z=zpath[next]
    
        returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
        Arro=newObjectHandles[0]
        #vrep.simxSetObjectPosition (clientID,Arro,-1,(0,0,0),vrep.simx_opmode_oneshot)
        #returnCode=vrep.simxSetObjectOrientation(clientID,Arro,-1,[angle_calculationx([0,bdiff,cdiff],[0,0,-1]),-angle_calculationy([0,bdiff,cdiff],[adiff,bdiff,cdiff]),0],vrep.simx_opmode_oneshot) #winkel_berechnen([0,bdiff,cdiff],[adiff,bdiff,cdiff])
        vrep.simxSetObjectPosition (clientID,Arro,-1,(x,y,z),vrep.simx_opmode_oneshot)  

def show_path2(path2,clientID):
    errorCode,Ball=vrep.simxGetObjectHandle(clientID,'A_star_points',vrep.simx_opmode_oneshot_wait)
    objectHandles=np.array([Ball])
    xpath=path2[0]
    ypath=path2[1]
    zpath=path2[2]
    for next in range(len(xpath)):
        x=xpath[next]
        y=ypath[next]
        z=zpath[next]
        returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
        Ball_new=newObjectHandles[0]
        vrep.simxSetObjectPosition (clientID,Ball_new,-1,(x,y,z),vrep.simx_opmode_oneshot)
        
def followPath(clientID,path):
    errorCode,UAV=vrep.simxGetObjectHandle(clientID,'UAV_target',vrep.simx_opmode_oneshot_wait)
    for next in range(200):
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
        
def followPath2(clientID,path,goal):
    pathx=path[0]
    pathy=path[1]
    pathz=path[2]
    errorCode,UAV=vrep.simxGetObjectHandle(clientID,'UAV',vrep.simx_opmode_oneshot_wait)
    goal=UAV_pathfinding_astar.m_to_grid(goal)
    (xgoal,ygoal,zgoal)=goal
    errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_streaming)
    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_streaming)
    time.sleep(0.1) 
    errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
    #pos=getPosition(clientID,'UAV')
    #pos=UAV_pathfinding_astar.m_to_grid(pos)
    xPosition=pos[0]
    yPosition=pos[1]
    zPosition=pos[2]
    xvelomax=0.45
    yvelomax=0.45
    zmax=1
    vecp=[0,0,0]
    while (xPosition > pathx[199]+0.2) or (xPosition < pathx[199]-0.2) or (yPosition > pathy[199]+0.2) or (yPosition < pathy[199]-0.2) or (zPosition > pathz[199]+0.2) or (zPosition < pathz[199]-0.2):
        #start_time = time.time()
        #pos=getPosition(clientID,'UAV')
        errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
        errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
        #pos=UAV_pathfinding_astar.m_to_grid(pos)
        xPosition=pos[0]
        yPosition=pos[1]
        zPosition=pos[2]       
        #print("--- %s seconds ---" % (time.time() - start_time))
               
        vec=pathfollowing.findnearst(pos,path)
        #print vec
        #print vecp
        vecp=vec 
        
        absolut=math.sqrt(vec[0]**2+vec[1]**2+vec[2]**2)
        xvelo=0
        yvelo=0
        height=zPosition
    
        xvelo_w=xvelomax*vec[0]/absolut#ref_velx
       
        yvelo_w=yvelomax*vec[1]/absolut#ref_vely
        
        height=zPosition+vec[2]*zmax            #e
        
        #print height
        #ref_angz, angle between (1/0/0) and (xvelo/yvelo/0)
        a1=[1,0,0]
        #b1=[xvelo,yvelo,0]
        b1=[xvelo_w,yvelo_w,0]
        ref_angz=angle_calculationx(a1,b1)    
        if orientation[2]<0:
            angle=2*np.pi+orientation[2]
        else:
            angle=orientation[2]
        if ref_angz<0:
            ref_angz=2*np.pi+ref_angz
        if angle>ref_angz:
            veloz=-2*(angle-ref_angz)/np.pi
        else:
            veloz=2*(ref_angz-angle)/np.pi
        #ref_angz=0
        xvelo=xvelo_w*np.cos(-orientation[2])-yvelo_w*np.sin(-orientation[2])
        yvelo=xvelo_w*np.sin(-orientation[2])+yvelo_w*np.cos(-orientation[2])
        data=[xvelo,yvelo,height,0,0,veloz]
        #data=[-0.1,0,height,0,0,veloz]
        #print data
        packedData=vrep.simxPackFloats(data)
        vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
        #time.sleep(0.5)
        
    data=[0,0,height,0,0,0]
    packedData=vrep.simxPackFloats(data)
    vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
    