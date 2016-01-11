# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 15:31:37 2015

@author: lijinke
"""
import vrep#needed for the Connection with the Simulator
import numpy as np#needed for the arrays and some other mathematical operations
import time
import math
import pathfollowing

#get the position of an object
def getPosition(clientID,object_name):
    #get the handle of the object, which position is needed
    errorcode,object_handle=vrep.simxGetObjectHandle(clientID,object_name,vrep.simx_opmode_oneshot_wait)
    #get the position
    errorCode,object_position=vrep.simxGetObjectPosition(clientID,object_handle,-1,vrep.simx_opmode_streaming)
    #some waiting time is needed to get the right data    
    time.sleep(1) 
    errorCode,object_position=vrep.simxGetObjectPosition(clientID,object_handle,-1,vrep.simx_opmode_buffer)
    return object_position

#calculate the angle between 2 vectors a and b
def angle_calculation(a,b):
    dot = np.dot(a,b)
    x_modulus = np.sqrt(a[0]**2+a[1]**2+a[2]**2)
    y_modulus = np.sqrt(b[0]**2+b[1]**2+b[2]**2)
    cos_angle = dot / x_modulus / y_modulus 
    angle = np.arccos(cos_angle) #angle in radiant
    if b[1]>0:
        return angle
    else:
        return -angle

#show the path in V-REP
def show_path(path,clientID):
    #make a signal for each path-component
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
    #signals that the data of the path is ready to be read
    data2=[1]
    packedData2=vrep.simxPackFloats(data2)
    vrep.simxClearStringSignal(clientID,'Command_Path_Ready',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Command_Path_Ready',packedData2,vrep.simx_opmode_oneshot)
        
        
def followPath(clientID,path,goal):
    pathx=path[0]
    pathy=path[1]
    pathz=path[2]
    errorCode,UAV=vrep.simxGetObjectHandle(clientID,'UAV',vrep.simx_opmode_oneshot_wait)
    errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_streaming)
    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_streaming)
    errorCode,gesl,gesa=vrep.simxGetObjectVelocity(clientID,UAV,vrep.simx_opmode_streaming)
    time.sleep(0.1) 
    errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
    errorCode,gesl,gesa=vrep.simxGetObjectVelocity(clientID,UAV,vrep.simx_opmode_buffer)
    
    #some initialization
    xPosition=pos[0]
    yPosition=pos[1]
    zPosition=pos[2]
    xp=[]
    yp=[]
    zp=[]
    xpnear=[]
    ypnear=[]
    zpnear=[]
    xerror=[]
    yerror=[]
    zerror=[]
    pdangle=0
    pveloz=0
    pangle=0
    pref_angz=0
    distest=1
    
    while (distest > 0.05):
        xvelomax=1.5
        yvelomax=1.5
        turnmax=8
        absolut_dis=math.sqrt((xPosition-pathx[(len(pathx)-1)])**2+(yPosition-pathy[(len(pathx)-1)])**2+(zPosition-pathz[(len(pathx)-1)])**2)

        slowvelo_dis=4
        if absolut_dis < slowvelo_dis:
            xvelomax=(xvelomax-0.15)*absolut_dis/slowvelo_dis*absolut_dis/slowvelo_dis+0.15
            yvelomax=(yvelomax-0.15)*absolut_dis/slowvelo_dis*absolut_dis/slowvelo_dis+0.15
        
        errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
        errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
        errorCode,gesl,gesa=vrep.simxGetObjectVelocity(clientID,UAV,vrep.simx_opmode_buffer)
       
        xPosition=pos[0]
        yPosition=pos[1]
        zPosition=pos[2]       
        
        xp.append(xPosition)
        yp.append(yPosition)
        zp.append(zPosition)
               
        vec,vec_path,pnear,dis=pathfollowing.findnearst(pos,path)

        xpnear.append(pnear[0])
        ypnear.append(pnear[1])
        zpnear.append(pnear[2])
        
        xerror.append(abs(xPosition-pnear[0]))
        yerror.append(abs(yPosition-pnear[1]))
        zerror.append(abs(zPosition-pnear[2]))
        
        (xgoal,ygoal,zgoal)=goal
        distest=np.sqrt((xPosition-pathx[(len(pathx)-1)])**2+(yPosition-pathy[(len(pathx)-1)])**2)
      
        
        absolut=math.sqrt(vec[0,0]**2+vec[0,1]**2)
        xvelo=0
        yvelo=0
        height=zPosition
    
        xvelo_w=xvelomax*vec[0,0]/absolut#ref_velx

        yvelo_w=yvelomax*vec[0,1]/absolut#ref_vely
        
        height=zPosition+vec[0,2]      #e
        
        #print height
        #ref_angz, angle between (1/0/0) and (xvelo/yvelo/0)
        a1=[1,0,0]
        b1=[vec_path[0],vec_path[1],0]
        #b1=[xvelo_w,yvelo_w,0]
        ref_angz=angle_calculation(a1,b1)    
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
        #xvelo_w=xvelo_w*((np.pi-abs(dangle))/np.pi)**20
        #yvelo_w=yvelo_w*((np.pi-abs(dangle))/np.pi)**20
        #xvelo=xvelo_w*np.cos(ref_angz)+yvelo_w*np.sin(ref_angz)
        #yvelo=yvelo_w*np.cos(ref_angz)-xvelo_w*np.sin(ref_angz)
        xvelo=xvelo_w*np.cos(angle)+yvelo_w*np.sin(angle)
        yvelo=yvelo_w*np.cos(angle)-xvelo_w*np.sin(angle)
        #print dis
        if abs(vec[0,2])>0.2:
            xvelo=0
            yvelo=0
        else:
            if dis<0.5:
                xvelo=xvelo*((np.pi-abs(dangle))/np.pi)**130*(((0.2-abs(vec[0,2]))/0.2)**6)*((0.5-dis)/0.5)
                yvelo=yvelo*((np.pi-abs(dangle))/np.pi)**15*(((0.2-abs(vec[0,2]))/0.2)**6)*(dis+0.2)**2
            else:
                xvelo=xvelo*((np.pi-abs(dangle))/np.pi)**130*(((0.2-abs(vec[0,2]))/0.2)**6)*((0.5-dis)/0.5)
                yvelo=yvelo*((np.pi-abs(dangle))/np.pi)**15*(((0.2-abs(vec[0,2]))/0.2)**6)
        if abs(vec_path[2])>0.003:
            print vec_path[2]
            xvelo=xvelo*(0.003/abs(vec_path[2]))**3
            yvelo=yvelo*(0.003/abs(vec_path[2]))
            height=height+vec[0,2]*4
        data=[xvelo,yvelo,height,0,0,veloz]
        #data=[-0.1,0,height,0,0,veloz]
        #print data
        #print xvelo,yvelo,veloz,gesl
        packedData=vrep.simxPackFloats(data)
        vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
        time.sleep(0.15)
        vrep.simxSynchronousTrigger(clientID);
        
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
   
   