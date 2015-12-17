# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 21:00:37 2015

@author: renchen
"""
import numpy as np
import time
def  findnearst(position, path):
    
    x=path[0]
    y=path[1]
    z=path[2]
    dx=position[0]-x
    dy=position[1]-y
    dz=position[2]-z
         
    distance=np.sqrt(dx*dx+dy*dy+dz*dz)  #caculate distance between position and every points in the path

    minindex=np.argmin(distance)
    print minindex    
         #print "nearstpoint:",np.transpose(path)[minindex]
    p_near=np.transpose(path)[minindex]  #get the nearst point

    
         
    v_approx=p_near-position
    #v_approx_nor=v_approx/(np.sqrt(v_approx[0]*v_approx[0]+v_approx[1]*v_approx[1]+v_approx[2]*v_approx[2]))
    if minindex<50:
        p_near1=np.transpose(path)[minindex+1] # the point which next to the nearst point
        v_tangent = p_near1-p_near
        #v_tangent_nor=v_tangent/(np.sqrt(v_tangent[0]*v_tangent[0]+v_tangent[1]*v_tangent[1]+v_tangent[2]*v_tangent[2]))
        alfa=np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])    # 0<alfa<1  self defination
        beta=np.matrix([[100, 0, 0],[0, 100, 0],[0, 0, 0]])     # 0<beta<1  self defination
        #v_result = alfa*v_approx_nor + beta*v_tangent_nor
        v_result = v_approx*alfa + v_tangent*beta
        #print v_approx,v_tangent,v_result
    else:
        if minindex<(len(x)-51):
            p_near1=np.transpose(path)[minindex+1] # the point which next to the nearst point
            v_tangent = p_near1-p_near
            #v_tangent_nor=v_tangent/(np.sqrt(v_tangent[0]*v_tangent[0]+v_tangent[1]*v_tangent[1]+v_tangent[2]*v_tangent[2]))
            alfa=np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])    # 0<alfa<1  self defination
            beta=np.matrix([[50, 0, 0],[0, 50, 0],[0, 0, 0]])     # 0<beta<1  self defination
            #v_result = alfa*v_approx_nor + beta*v_tangent_nor
            v_result = v_approx*alfa + v_tangent*beta
            #print v_approx,v_tangent,v_result
        else:
            if minindex<(len(x)-1):
                p_near1=np.transpose(path)[minindex+1] # the point which next to the nearst point
                v_tangent = p_near1-p_near
                #v_tangent_nor=v_tangent/(np.sqrt(v_tangent[0]*v_tangent[0]+v_tangent[1]*v_tangent[1]+v_tangent[2]*v_tangent[2]))
                alfa=np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])    # 0<alfa<1  self defination
                beta=np.matrix([[20, 0, 0],[0, 20, 0],[0, 0, 0]])     # 0<beta<1  self defination
                #v_result = alfa*v_approx_nor + beta*v_tangent_nor
                v_result = v_approx*alfa + v_tangent*beta
                #print v_approx,v_tangent,v_result
                print 'Near'
            else:
                print 'GOAL'
                alfa=np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])     # 0<alfa<1  self defination
                #beta=15     # 0<beta<1  self defination
                #v_result = alfa*v_approx_nor
                v_result = v_approx*alfa
    return v_result,p_near


     