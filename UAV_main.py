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

##generate mapdata, load data if mapdata for scene exist not implemented till now   
#mapdata=UAV_mapgen.mapgen_fast("hexagon_new2",44,44,20,clientID)
##mapdata=UAV_mapgen.mapgen("testroom111",12,12,4,clientID)
#
##start the synchron-simulation-mode to improve the communication between python and V-REP, while the simulation is running
#vrep.simxSynchronous(clientID,True); 
#
##Get goal-data from V-REP
#goal_position=UAV_VREP.getPosition(clientID,'goal_new')
#print goal_position
#   
#
##Get start-data from v-REP
#start_position=UAV_VREP.getPosition(clientID,'UAV')
#print start_position
#             
##Start pathfinding
#print "start pathfinding"
#
#path=UAV_pathfinding.search(goal_position,start_position,"astar",3,mapdata)
#print "pathfinding finished"
#
##function to start the signals to transport the data to V-REP(LUA) and give the signal to the UAV-script, that the path is ready
#UAV_VREP.show_path(path,clientID)
#
##function to follow the path, generates the needed velocities and heights, that are needed for the LUA-script and streams the needed signals
#xyzarray,xyzneararray=UAV_VREP.followPath(clientID,path,goal_position)
#
## Plot
##arrange the data for the plot
#xerror=abs(xyzarray[0]-xyzneararray[0])
#yerror=abs(xyzarray[1]-xyzneararray[1])
#zerror=abs(xyzarray[2]-xyzneararray[2])
#xspace=np.linspace(0, np.size(xerror)-1, num=np.size(xerror))
#yspace=np.linspace(0, np.size(yerror)-1, num=np.size(yerror))
#zspace=np.linspace(0, np.size(zerror)-1, num=np.size(zerror))
#
##plot the path over the real movement in 2D(xy-plane)
#plt.plot(xyzarray[0],xyzarray[1], 'ro', path[0], path[1], 'g-')



#somemore plot-suggestions that are possible showing different aspects of the path-following
#plt.plot(xspace,xerror,'r',yspace,yerror,'g',zspace,zerror,'b')
#plt.plot(xspace,np.sqrt(xerror*xerror+yerror*yerror))

#plt.plot(yspace,yerror)
#plt.plot(xspace,xerror)


#plt.plot(xpnear,ypnear, 'r--', xp, yp, 'g')
#ax = plt.axes(projection='3d')
#ax.plot(path[0],path[1], path[2], '-b')

#figure(0)
#plt.plot(xspace,xerror,'r',yspace,yerror,'g',zspace,zerror,'b')
#figure(1)
#plt.plot(xspace,np.sqrt(xerror*xerror+yerror*yerror))
#figure(2)
#ax = plt.axes(projection='3d')
#ax.plot(path[0],path[1],path[2],'b')
#ax.plot(xyzarray[0],xyzarray[1],xyzarray[2],'r')
#
#plt.plot(yspace,yerror)
#plt.plot(xspace,xerror)
#
#plt.plot(path[0],path[1], 'r--', xp, yp, 'g')
#plt.plot(xpnear,ypnear, 'r--', xp, yp, 'g')
#ax = plt.axes(projection='3d')
#ax.plot(path[0],path[1], path[2], '-b')

#stepresponeses of height controller
#errorCode,UAV=vrep.simxGetObjectHandle(clientID,'UAV',vrep.simx_opmode_oneshot_wait)
#errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_streaming)
#errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_streaming)
#
#time_data1=[]
#height_data1=[]
#time_data2=[]
#height_data2=[]
#time_data3=[]
#height_data3=[]
#time_data4=[]
#height_data4=[]
#
#data=[0,0,-10,0,0,0]
#packedData=vrep.simxPackFloats(data)
#vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
#vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
#
#time.sleep(5)
#
#
#
#
#for i in range(500):
#    cmdTime=vrep.simxGetLastCmdTime(clientID)
#    errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
#    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
#    height_data1.append(pos[2])
#    time_data1.append(cmdTime)
#    time.sleep(0.01)
#    if i>50:
#        data=[0,0,-9.9,0,0,0]
#        packedData=vrep.simxPackFloats(data)
#        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
#        
#data=[0,0,-10,0,0,0]
#packedData=vrep.simxPackFloats(data)
#vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
#
#time.sleep(10)   
#
#
#for i in range(500):
#    cmdTime=vrep.simxGetLastCmdTime(clientID)
#    errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
#    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
#    height_data2.append(pos[2])
#    time_data2.append(cmdTime)
#    time.sleep(0.01)
#    if i>50:
#        data=[0,0,-9.75,0,0,0]
#        packedData=vrep.simxPackFloats(data)
#        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
#
#data=[0,0,-10,0,0,0]
#packedData=vrep.simxPackFloats(data)
#vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
#
#time.sleep(10)
#
#for i in range(500):
#    cmdTime=vrep.simxGetLastCmdTime(clientID)
#    errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
#    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
#    height_data3.append(pos[2])
#    time_data3.append(cmdTime)
#    time.sleep(0.01)
#    if i>50:
#        data=[0,0,-9.5,0,0,0]
#        packedData=vrep.simxPackFloats(data)
#        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
#
#data=[0,0,-10,0,0,0]
#packedData=vrep.simxPackFloats(data)
#vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
#
#time.sleep(10)
#        
#for i in range(500):
#    cmdTime=vrep.simxGetLastCmdTime(clientID)
#    errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
#    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
#    height_data4.append(pos[2])
#    time_data4.append(cmdTime)
#    time.sleep(0.01)
#    if i>50:
#        data=[0,0,-9,0,0,0]
#        packedData=vrep.simxPackFloats(data)
#        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
#
#
#time_data1p=time_data1[1:]
#time_data2p=time_data2[1:]
#time_data3p=time_data3[1:]
#time_data4p=time_data4[1:]
#
#height_data1p=height_data1[1:]
#height_data2p=height_data2[1:]
#height_data3p=height_data3[1:]
#height_data4p=height_data4[1:]
#
#for i in range(499):
#    time_data1p[i]=time_data1p[i]-time_data1[1]
#    time_data2p[i]=time_data2p[i]-time_data2[1]
#    time_data3p[i]=time_data3p[i]-time_data3[1]
#    time_data4p[i]=time_data4p[i]-time_data4[1]
#for i in range(499):
#    height_data1p[i]=height_data1p[i]-height_data1[1]
#    height_data2p[i]=height_data2p[i]-height_data2[1]
#    height_data3p[i]=height_data3p[i]-height_data3[1]
#    height_data4p[i]=height_data4p[i]-height_data4[1]
#
#plt.plot(time_data1p,height_data1p,'r')
#plt.plot(time_data2p,height_data2p,'b')
#plt.plot(time_data3p,height_data3p,'g')
#plt.plot(time_data4p,height_data4p,'y')
#plt.xlabel('$t$ in ms',fontsize = 20)
#plt.ylabel('$x_3$ in m',fontsize = 20)
#plt.xticks(fontsize = 15)
#plt.yticks(fontsize = 15)
#plt.ylim(0, 1.1)
#plt.xlim(0, 7500)

errorCode,UAV=vrep.simxGetObjectHandle(clientID,'UAV',vrep.simx_opmode_oneshot_wait)
errorCode,velol,veloa=vrep.simxGetObjectVelocity(clientID,UAV,vrep.simx_opmode_streaming)
errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_streaming)

time_data1=[]
height_data1=[]
time_data2=[]
height_data2=[]
time_data3=[]
height_data3=[]
time_data4=[]
height_data4=[]

data=[0,0,-10,0,0,0]
packedData=vrep.simxPackFloats(data)
vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)

time.sleep(5)




for i in range(500):
    cmdTime=vrep.simxGetLastCmdTime(clientID)
    errorCode,velol,veloa=vrep.simxGetObjectVelocity(clientID,UAV,vrep.simx_opmode_buffer)
    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
    height_data1.append(velol[1])
    time_data1.append(cmdTime)
    time.sleep(0.01)
    if i>50:
        data=[0,0.1,-10,0,0,0]
        packedData=vrep.simxPackFloats(data)
        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)
        
data=[0,0,-10,0,0,0]
packedData=vrep.simxPackFloats(data)
vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)

time.sleep(10)   


for i in range(500):
    cmdTime=vrep.simxGetLastCmdTime(clientID)
    errorCode,velol,veloa=vrep.simxGetObjectVelocity(clientID,UAV,vrep.simx_opmode_buffer)
    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
    height_data2.append(velol[1])
    time_data2.append(cmdTime)
    time.sleep(0.01)
    if i>50:
        data=[0,0.3,-10,0,0,0]
        packedData=vrep.simxPackFloats(data)
        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)

data=[0,0,-10,0,0,0]
packedData=vrep.simxPackFloats(data)
vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)

time.sleep(10)

for i in range(500):
    cmdTime=vrep.simxGetLastCmdTime(clientID)
    errorCode,velol,veloa=vrep.simxGetObjectVelocity(clientID,UAV,vrep.simx_opmode_buffer)
    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
    height_data3.append(velol[1])
    time_data3.append(cmdTime)
    time.sleep(0.01)
    if i>50:
        data=[0,0.75,-10,0,0,0]
        packedData=vrep.simxPackFloats(data)
        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)

data=[0,0,-10,0,0,0]
packedData=vrep.simxPackFloats(data)
vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)

time.sleep(10)
        
for i in range(500):
    cmdTime=vrep.simxGetLastCmdTime(clientID)
    errorCode,velol,veloa=vrep.simxGetObjectVelocity(clientID,UAV,vrep.simx_opmode_buffer)
    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
    height_data4.append(velol[1])
    time_data4.append(cmdTime)
    time.sleep(0.01)
    if i>50:
        data=[0,1.5,-10,0,0,0]
        packedData=vrep.simxPackFloats(data)
        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)


time_data1p=time_data1[1:]
time_data2p=time_data2[1:]
time_data3p=time_data3[1:]
time_data4p=time_data4[1:]

height_data1p=height_data1[1:]
height_data2p=height_data2[1:]
height_data3p=height_data3[1:]
height_data4p=height_data4[1:]

for i in range(499):
    time_data1p[i]=time_data1p[i]-time_data1[1]
    time_data2p[i]=time_data2p[i]-time_data2[1]
    time_data3p[i]=time_data3p[i]-time_data3[1]
    time_data4p[i]=time_data4p[i]-time_data4[1]
for i in range(499):
    height_data1p[i]=height_data1p[i]-height_data1[1]
    height_data2p[i]=height_data2p[i]-height_data2[1]
    height_data3p[i]=height_data3p[i]-height_data3[1]
    height_data4p[i]=height_data4p[i]-height_data4[1]

plt.plot(time_data1p,height_data1p,'r')
plt.plot(time_data2p,height_data2p,'b')
plt.plot(time_data3p,height_data3p,'g')
plt.plot(time_data4p,height_data4p,'y')
plt.xlabel('$t$ in ms',fontsize = 20)
plt.ylabel('$\dot{x}_2$ in m',fontsize = 20)
plt.xticks(fontsize = 15)
plt.yticks(fontsize = 15)
plt.ylim(0, 1.6)
plt.xlim(0, 7500)