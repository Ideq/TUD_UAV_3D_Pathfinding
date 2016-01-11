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
    #arrange the data given to this function
    pathx=path[0]
    pathy=path[1]
    pathz=path[2]
    #get the start infos for the following
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
    absolut_dis=1
    #path-following loop
    #here is the goal defined, in this case reach the goal <5cm, <2cm also works in nearly all case, but needs some more time at the end
    while (absolut_dis > 0.05):
        #define the max possible velocities
        xvelomax=1.5
        yvelomax=1.5
        #define the max possible turnrate
        turnmax=8
        #calculate the distance to the goal
        absolut_dis=math.sqrt((xPosition-pathx[(len(pathx)-1)])**2+(yPosition-pathy[(len(pathx)-1)])**2+(zPosition-pathz[(len(pathx)-1)])**2)
        #define the radius around the goal, in which the UAV is slowed to lower the error between UAV and path
        slowvelo_dis=4
        #lower the max velocities in the defined radius, closer to the goal the UAV is more slowed
        if absolut_dis < slowvelo_dis:
            xvelomax=(xvelomax-0.15)*absolut_dis/slowvelo_dis*absolut_dis/slowvelo_dis+0.15
            yvelomax=(yvelomax-0.15)*absolut_dis/slowvelo_dis*absolut_dis/slowvelo_dis+0.15
        #refresh the UAV information
        errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
        errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
        errorCode,gesl,gesa=vrep.simxGetObjectVelocity(clientID,UAV,vrep.simx_opmode_buffer)
        #arrange the information
        xPosition=pos[0]
        yPosition=pos[1]
        zPosition=pos[2]       
        
        #save some information for the documentation after the following
        xp.append(xPosition)
        yp.append(yPosition)
        zp.append(zPosition)
        
        #get the direction the UAV should fly to follow the path
        vec,vec_path,pnear,dis=pathfollowing.findnearst(pos,path)
        
        #some more info for documentation
        xpnear.append(pnear[0])
        ypnear.append(pnear[1])
        zpnear.append(pnear[2])
        
        xerror.append(abs(xPosition-pnear[0]))
        yerror.append(abs(yPosition-pnear[1]))
        zerror.append(abs(zPosition-pnear[2]))
        
        #calculate the velocities from the direction vector
        absolut=math.sqrt(vec[0,0]**2+vec[0,1]**2)
        xvelo=0
        yvelo=0
        height=zPosition
        xvelo_w=xvelomax*vec[0,0]/absolut#ref_velx
        yvelo_w=yvelomax*vec[0,1]/absolut#ref_vely
        height=zPosition+vec[0,2]      #e
        
        #ref_angz, angle between (1/0/0) and (xvelo/yvelo/0)
        a1=[1,0,0]
        b1=[vec_path[0],vec_path[1],0]
        ref_angz=angle_calculation(a1,b1)    
        
        #arrange some info
        angle=orientation[2]
        dangle=ref_angz-angle
        
        #angle correction to prevent the UAV from turning not into the shorter direction
        if dangle>np.pi:
            dangle=dangle-2*np.pi
        if dangle<-np.pi:
            dangle=2*np.pi+dangle
        #calculate the turnrate, its lower, if the orientation error is lower, to prevent the UAV from doing a oszillation around the right orientation
        veloz=turnmax*(dangle)/np.pi
       
        #transfom the velocities into the UAV-coordinate-system from the world-coordinates
        xvelo=xvelo_w*np.cos(angle)+yvelo_w*np.sin(angle)
        yvelo=yvelo_w*np.cos(angle)-xvelo_w*np.sin(angle)
        
        #modify the velocities to improve the path following, by using the current distances and errors to the path and his orientation
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
        
        #create the data-signal, which is read to the LUA-script
        data=[xvelo,yvelo,height,0,0,veloz]
        packedData=vrep.simxPackFloats(data)
        vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
        
        #trigger the next simulation step
        vrep.simxSynchronousTrigger(clientID);
    
    #clean up some signals
    data=[0,0,pathz[(len(pathx)-1)],0,0,0]
    packedData=vrep.simxPackFloats(data)
    vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
    
    data2=[0]
    packedData2=vrep.simxPackFloats(data2)
    vrep.simxClearStringSignal(clientID,'Command_Path_Ready',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Command_Path_Ready',packedData2,vrep.simx_opmode_oneshot)    
    
    #prepare some data for the plot of the path
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
        
    #return plot data   
    return xyzparray,xyzneararray
   
   