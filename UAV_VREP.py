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

def show_path2(path,clientID):
#    errorCode,Ball=vrep.simxGetObjectHandle(clientID,'A_star_points',vrep.simx_opmode_oneshot_wait)
#    objectHandles=np.array([Ball])
#    xpath=path2[0]
#    ypath=path2[1]
#    zpath=path2[2]
#    for next in range(len(xpath)):
#        x=xpath[next]
#        y=ypath[next]
#        z=zpath[next]
#        returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
#        Ball_new=newObjectHandles[0]
#        vrep.simxSetObjectPosition (clientID,Ball_new,-1,(x,y,z),vrep.simx_opmode_oneshot)
    datax=path[0]
    datay=path[1]
    dataz=path[2]
    packedDatax=vrep.simxPackFloats(datax)
    vrep.simxClearStringSignal(clientID,'Path_Signalx',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Path_Signalx',packedDatax,vrep.simx_opmode_oneshot)
    packedDatay=vrep.simxPackFloats(datay)
    vrep.simxClearStringSignal(clientID,'Path_Signaly',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Path_Signaly',packedDatay,vrep.simx_opmode_oneshot)
    packedDataz=vrep.simxPackFloats(dataz)
    vrep.simxClearStringSignal(clientID,'Path_Signalz',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Path_Signalz',packedDataz,vrep.simx_opmode_oneshot)
        
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
    #goal=UAV_pathfinding_astar.m_to_grid(goal)
    #(xgoal,ygoal,zgoal)=goal
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
    xvelomax=0.3
    yvelomax=0.3
    zmax=1
    xp=[]
    yp=[]
    zp=[]
    xpnear=[]
    ypnear=[]
    zpnear=[]
    xerror=[]
    yerror=[]
    zerror=[]

    vecp=[0,0,0]
    pdangle=0
    pveloz=0
    pangle=0
    pref_angz=0
    distest=1
    #while (xPosition > pathx[(len(pathx)-1)]+0.1) or (xPosition < pathx[(len(pathx)-1)]-0.1) or (yPosition > pathy[(len(pathx)-1)]+0.1) or (yPosition < pathy[(len(pathx)-1)]-0.1):
    while (distest > 0.05):
        #xvelomax=0.23
        #yvelomax=0.23
        xvelomax=0.5
        yvelomax=0.5
        zmax=1
        turnmax=5
        absolut_dis=math.sqrt((xPosition-pathx[(len(pathx)-1)])**2+(yPosition-pathy[(len(pathx)-1)])**2+(zPosition-pathz[(len(pathx)-1)])**2)
        absolut_dis2=math.sqrt((xPosition-pathx[0])**2+(yPosition-pathy[0])**2+(zPosition-pathz[0])**2)
        #print absolut_dis
        slowvelo_dis=4
        if absolut_dis < slowvelo_dis:
            xvelomax=(xvelomax-0.1)*absolut_dis/slowvelo_dis*absolut_dis/slowvelo_dis+0.1
            yvelomax=(yvelomax-0.1)*absolut_dis/slowvelo_dis*absolut_dis/slowvelo_dis+0.1
            #turnmax=turnmax*absolut_dis/slowvelo_dis
        if absolut_dis2 < slowvelo_dis/4:
            xvelomax=(xvelomax-0.1)*absolut_dis2/slowvelo_dis/4+0.1
            yvelomax=(yvelomax-0.1)*absolut_dis2/slowvelo_dis/4+0.1
            #turnmax=2
            #zmax=zmax*absolut_dis/slowvelo_dis
            
            #print xvelomax
            #print yvelomax
            #print zmax   
            
        #pos=getPosition(clientID,'UAV')
        errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
        errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
        #pos=UAV_pathfinding_astar.m_to_grid(pos)
        xPosition=pos[0]
        yPosition=pos[1]
        zPosition=pos[2]       
        
        xp.append(xPosition)
        yp.append(yPosition)
        zp.append(zPosition)
               
        vec,pnear=pathfollowing.findnearst(pos,path)
        #vec=vec1[0]        
        #print vec
        xpnear.append(pnear[0])
        ypnear.append(pnear[1])
        zpnear.append(pnear[2])
        
        xerror.append(abs(xPosition-pnear[0]))
        yerror.append(abs(yPosition-pnear[1]))
        zerror.append(abs(zPosition-pnear[2]))
        
        (xgoal,ygoal,zgoal)=goal
        distest=np.sqrt((xPosition-pathx[(len(pathx)-1)])**2+(yPosition-pathy[(len(pathx)-1)])**2)
        #xdiff=xPosition-pnear[0]
        #ydiff=yPosition-pnear[1]
        #print distest
        #print vec
        #print vecp
        #vecp=vec 
        
        absolut=math.sqrt(vec[0,0]**2+vec[0,1]**2)
        xvelo=0
        yvelo=0
        height=zPosition
    
        xvelo_w=xvelomax*vec[0,0]/absolut#ref_velx

        yvelo_w=yvelomax*vec[0,1]/absolut#ref_vely
        
        height=zPosition+vec[0,2]*zmax            #e
        
        #print height
        #ref_angz, angle between (1/0/0) and (xvelo/yvelo/0)
        a1=[1,0,0]
        #b1=[xvelo,yvelo,0]
        b1=[xvelo_w,yvelo_w,0]
        ref_angz=angle_calculationx(a1,b1)    
        #if orientation[2]<0:
            #angle=2*np.pi+orientation[2]
        #else:
        angle=orientation[2]
        #if ref_angz<0:
            #ref_angz=2*np.pi+ref_angz
        dangle=ref_angz-angle
        #print dangle
        #if dangle>np.pi:
            #print dangle
        if dangle>np.pi:
            dangle=dangle-2*np.pi
        if dangle<-np.pi:
            dangle=2*np.pi+dangle
        veloz=turnmax*(dangle)/np.pi
        #print dangle
        #ref_angz=0
        #if dangle-pdangle>1:
            #print pdangle,dangle
            #print pangle,angle
            #print pref_angz,ref_angz
            #print pveloz,veloz
        pdangle=dangle
        pveloz=veloz
        pangle=angle
        pref_angz=ref_angz
        #if (xvelo_w<0.01) and (yvelo_w<0.01):
            #veloz=0
        xvelo_w=xvelo_w*((np.pi-abs(dangle))/np.pi)**20
        yvelo_w=yvelo_w*((np.pi-abs(dangle))/np.pi)**20
        xvelo=xvelo_w*np.cos(ref_angz)+yvelo_w*np.sin(ref_angz)
        yvelo=yvelo_w*np.cos(ref_angz)-xvelo_w*np.sin(ref_angz)
        #xvelo=xvelo_w*np.cos(angle)+yvelo_w*np.sin(angle)
        #yvelo=yvelo_w*np.cos(angle)-xvelo_w*np.sin(angle)
        data=[xvelo,yvelo,height,0,0,veloz]
        #data=[-0.1,0,height,0,0,veloz]
        #print data
        print xvelo_w,yvelo_w
        packedData=vrep.simxPackFloats(data)
        vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
        #time.sleep(1) 
    data=[0,0,zgoal,0,0,0]
    packedData=vrep.simxPackFloats(data)
    vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
    
    
    data2=[0]
    packedData2=vrep.simxPackFloats(data2)
    vrep.simxClearStringSignal(clientID,'Command_Path_Ready',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Command_Path_Ready',packedData2,vrep.simx_opmode_oneshot)    
    
    xyzparray=np.ndarray(shape=(3,len(xp)),dtype=float)
    for next in range(len(xp)):
        xyzparray[0,next]=xp[next]
        xyzparray[1,next]=yp[next]
        xyzparray[2,next]=zp[next]
        
    xyzneararray=np.ndarray(shape=(3,len(xpnear)),dtype=float)
    for next in range(len(xpnear)):
        xyzneararray[0,next]=xpnear[next]
        xyzneararray[1,next]=ypnear[next]
        xyzneararray[2,next]=zpnear[next]
        
        
    return xyzparray,xyzneararray
   
   