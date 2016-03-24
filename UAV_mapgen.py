# -*- coding: utf-8 -*-
"""
Created on Fri Oct 30 14:52:27 2015
@author: lijinke
"""



import vrep
#import sys
import numpy as np
import time
import math
#from scipy import interpolate
from copy import deepcopy
import os.path
import cPickle as pickle

#function to extend the obstacles
def save(mapdata2):
    mapdata3=deepcopy(mapdata2)
    (x,y,z)=mapdata3.shape
    for index in range(1,x-1):
        for index2 in range(1,y-1):
            for index3 in range(1,z-1):
                if  ((mapdata2[index-1,index2+1,index3]==1) or (mapdata2[index-1,index2,index3+1]==1) or (mapdata2[index-1,index2,index3-1]==1) or(mapdata2[index-1,index2,index3]==1) or (mapdata2[index-1,index2-1,index3+1]==1) or (mapdata2[index-1,index2-1,index3-1]==1) or (mapdata2[index-1,index2-1,index3]==1) or (mapdata2[index,index2+1,index3+1]==1) or (mapdata2[index,index2+1,index3-1]==1) or (mapdata2[index,index2-1,index3]==1) or (mapdata2[index,index2,index3+1]==1) or (mapdata2[index,index2,index3-1]==1) or (mapdata2[index+1,index2-1,index3]==1) or (mapdata2[index,index2-1,index3+1]==1) or (mapdata2[index,index2-1,index3-1]==1) or (mapdata2[index+1,index2,index3]==1) or (mapdata2[index+1,index2+1,index3+1]==1) or (mapdata2[index+1,index2+1,index3-1]==1) or (mapdata2[index+1,index2+1,index3]==1) or (mapdata2[index+1,index2,index3+1]==1) or (mapdata2[index+1,index2,index3-1]==1) or (mapdata2[index,index2+1,index3]==1) or (mapdata2[index+1,index2-1,index3+1]==1) or (mapdata2[index+1,index2-1,index3-1]==1) or (mapdata2[index-1,index2+1,index3+1]==1) or (mapdata2[index-1,index2+1,index3-1]==1)):
                    mapdata3[index,index2,index3]=1
                    #print 'check'
    return mapdata3

#function to load or create the mapdata, depending on if it already exists or not
def mapgen(scene_name,x,y,z,clientID):
    #part which loads
    if os.path.isfile('./Mapdata/'+ scene_name +'.npy'):
        arr=np.load('./Mapdata/'+ scene_name +'.npy')
    #part which creates
    else:
        #initialize the 2 sensors
        errorCode,sensor1=vrep.simxGetObjectHandle(clientID,'Sensor_1',vrep.simx_opmode_oneshot_wait)
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor1,vrep.simx_opmode_streaming)            

        errorCode,sensor2=vrep.simxGetObjectHandle(clientID,'Sensor_2',vrep.simx_opmode_oneshot_wait)
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor2,vrep.simx_opmode_streaming)            
        #init some values
        time.sleep(3)
        xmax=x/0.4
        ymax=y/0.4 
        zmax=z/0.4                        
        xmax=int(math.ceil(xmax))
        ymax=int(math.ceil(ymax))
        zmax=int(math.ceil(zmax))
        arr=np.ndarray(shape=(xmax,ymax,zmax),dtype=int)
        #move the sensors through the map and create the data
        for index in range(xmax):
            for index2 in range(ymax):
                for index3 in range(zmax):
                    #move sensor
                    x=0.4+0.4*index
                    y=0.2+0.4*index2
                    z=0.3+0.4*index3
                    vrep.simxSetObjectPosition (clientID,sensor1,-1,(x,y,z),vrep.simx_opmode_oneshot)
                    vrep.simxSetObjectPosition (clientID,sensor2,-1,(x-0.4,y,z),vrep.simx_opmode_oneshot)
                    time.sleep(0.2)            
                    #check for collision
                    errorCode,detectionState1,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor1,vrep.simx_opmode_buffer)            
                    errorCode,detectionState2,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor2,vrep.simx_opmode_buffer)            
                    #save the collision result in an array 
                    if (detectionState1 or detectionState2):            
                        arr[index,index2,index3]=1   
                    else:
                        arr[index,index2,index3]=0 
        #save the final array as file
        np.save('./Mapdata/'+scene_name, np.ndarray(arr))
    return arr

#same concept as before but noch moving 100 sensors around to increase the speed of the mapgeneration    
def mapgen_fast(scene_name,x,y,z,clientID):
    #load mapdata if file exits
    if os.path.isfile('./Mapdata/'+scene_name+'.txt'):
        fo = open('./Mapdata/'+scene_name+'.txt', "r")
        data_string = fo.read();
        print "Read String is : ", str
        # Close opend file
        fo.close()
        arr = np.loads(data_string)
        return arr
    #otherwise generate mapdata
    else:
        x0=x
        y0=y
        #generate the 100 sensors and put them to theirstarting position
        errorCode,sensor1=vrep.simxGetObjectHandle(clientID,'Sensor_1',vrep.simx_opmode_oneshot_wait)
        objectHandles=np.array([sensor1])
        sensor_data=np.zeros(shape=(10,10,1),dtype=int)
        for x in range(10):
            for y in range(10):
                returnCode,new_sensor_handle=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
                vrep.simxReadProximitySensor(clientID,new_sensor_handle[0],vrep.simx_opmode_streaming)
                sensor_data[x,y]=new_sensor_handle[0]
                x1=0.4+x*0.4
                y1=0.2+y*0.4
                z1=0.21
                vrep.simxSetObjectPosition (clientID,sensor_data[x,y],-1,(x1,y1,z1),vrep.simx_opmode_oneshot)
        #init some values
        xmax=x0/4
        ymax=y0/4 
        zmax=z/0.4                       
        xmax=int(math.ceil(xmax))
        ymax=int(math.ceil(ymax))
        zmax=int(math.ceil(zmax))
        arr=np.zeros(shape=(xmax*10,ymax*10,zmax),dtype=int)
        #loop to detect the whole area
        for index in range(xmax):
            for index2 in range(ymax):
                for index3 in range(zmax):
                    for x in range(10):
                        for y in range(10):
                            errorCode,detectionState1,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor_data[x,y],vrep.simx_opmode_buffer) 
                            #save result                            
                            if (detectionState1):
                                arr[index*10+x,index2*10+y,index3]=1   
                            else:
                                arr[index*10+x,index2*10+y,index3]=0               
                    for x in range(10):
                        for y in range(10):
                            x1=0.4+4*(index)+x*0.4
                            y1=0.2+4*(index2)+y*0.4
                            z1=0.4*index3+0.21
                            vrep.simxSetObjectPosition (clientID,sensor_data[x,y],-1,(x1,y1,z1),vrep.simx_opmode_oneshot)    
                    time.sleep(1)
        #save mapdata to file
        arr=save(arr)
        data_string=np.ndarray.dumps(arr)
        print data_string
        fo = open('./Mapdata/'+scene_name+'.txt', "w")
        fo.write(data_string);
        fo.close()
        return arr
    