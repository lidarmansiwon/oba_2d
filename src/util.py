#!/usr/bin/env python3

import numpy as np
from math import *
import pandas  as pds
import math


def eulerFromQuaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return np.array([roll_x, pitch_y, yaw_z]) # in radians
                     


def change_atan(x,y):
    global psi_d
    if x < 0:
        if y < 0:
            psi_angle = atan(y/x)
            psi_d = (psi_angle - math.pi)
            return psi_d
        elif y > 0:
            psi_angle = atan(y/x)
            psi_d = (psi_angle + math.pi)
            return psi_d
    elif x == 0:
        psi_d = 0
        return psi_d
    elif x > 0:
        psi_d = atan(y/x)
        return float(psi_d)

def change_x(X,Y, Rot_CE_X,Rot_CE_Y,yaw): #X,Y = circle 
    rad = yaw
    x = (X-Rot_CE_X)*cos(rad) - (Y-Rot_CE_Y)*sin(rad) + Rot_CE_X
    y = (X-Rot_CE_Y)*sin(rad) + (Y-Rot_CE_Y)*cos(rad) + Rot_CE_Y
    return x
def change_y(X,Y, Rot_CE_X,Rot_CE_Y,yaw): #X,Y = circle 
    rad = yaw
    x = (X-Rot_CE_X)*cos(rad) - (Y-Rot_CE_Y)*sin(rad) + Rot_CE_X
    y = (X-Rot_CE_Y)*sin(rad) + (Y-Rot_CE_Y)*cos(rad) + Rot_CE_Y
    return y

def UTM_K_TRS(lati, longi):
    global la,lo

    # transformer = Transformer.from_crs("epsg:4326", "epsg:5178")
    # la, lo = transformer.transform(lati,longi)

    return [ la,lo] #동쪽 북쪽
    

def distance(x1,x2):    
    distance = sum([(x1[m]-x2[m])**2 for m in (0,1)])**0.5
    return distance

def tan2(x,y):
    tan2 = y/x
    return tan2

def mid_point_x(x1,x2):
    Mid_point_x = {(x1+x2)/2}
    return Mid_point_x

def mid_point_y(y1,y2):
    Mid_point_y = {(y1+y2)/2}
    return Mid_point_y

def circle(cp_x,cp_y,r,ang_first,ang,n):
    circle_dot=[[cp_x + r * cos(radians(i)), cp_y + r * sin(radians(i))] for i in range(ang_first,ang+1,n)]
    return  circle_dot

def new_street_line_end(x,y):
    a = atan(y/x)
    beta = math.pi/6
    k = tan(a+beta)
    return k

def To_close_line_end_(x,y):
    a = atan(y/x)
    beta = math.pi/2
    k = tan(a+beta)
    return k


def new_street_line_start(x,y):
    a = atan(y/x)
    beta = math.pi/6
    k = tan(a-beta)
    return k 

def To_close_line_start(x,y):
    a = atan(y/x)
    beta = math.pi/2
    k = tan(a-beta)
    return k

def cross_dot(x_cp,y_cp,x_m,y_m,k): # Point with circle and street 
    a=(1+k**2)
    b=2*(-x_cp-y_cp+k*x_m+y_m)
    c= x_cp**2 +(k*x_m+y_m-y_cp)**2
    dot=[]
    if b**2-4*a*c <0:
        pass
    elif b**2-4*a*c==0:
        x=-b/2
        y=k*(x-x_m)+y_m
        dot.append([x,y])

    else:
        x1 =  (-b+sqrt(b**2-4*a*c))/2
        x2 =(-b+sqrt(b**2-4*a*c))/2
        y1=k*(x1-x_m)+y_m
        y2=k*(x2-x_m)+y_m
        dot.append([x1,y1])
        dot.append([x2,y2])
    return dot