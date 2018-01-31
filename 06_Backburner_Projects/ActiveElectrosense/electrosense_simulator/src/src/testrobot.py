#!/usr/bin/env python

import rospy
import numpy as np
from math import atan2, pi
import hdt_nri_description.msg as hnd
import robot_calc_functions as r
import random

#Define lengths from tank to palm0.0
# Tank
# Fixed
# Roll of Pi/2
x_tank = 0
y_tank = 0
z_tank = 0

# Gantry Mounting Plate
# Fixed
x_mount = 0
y_mount = 0.952
z_mount = -0.5

# Gantry_x
# Prismatic
x_gantry_x = 0.0
y_gantry_x = 0.1035
z_gantry_x = 0.0

# Gantry_y
# Prismatic
x_gantry_y = 0.0
y_gantry_y = 0.1472
z_gantry_y = 0.0

# Gantry_z
# Prismatic
x_gantry_z = 0.1
y_gantry_z = 0.650
z_gantry_z = 0.161

# Gantry_Yaw
# Revolute
# ROLL OF PI!
x_gantry_yaw = 0.0
y_gantry_yaw = -1.223
z_gantry_yaw = 0.23

# Lengths of fingers
len_finger = 0.7 - 0.632
len_thumb = 0.77-0.64
len_palm = len_thumb*(5.5/4.5) #0.08 + 0.25

# Joint1
# R -2.094 to 2.094
x_j1 = 0.1-0.1
y_j1 = 0.26351 - 0.177
z_j1 = 0.62823 - 0.6297

# Joint2
# R -pi/2 to pi/2
x_j2 = 0.12393 - 0.1
y_j2 = 0.31451 - 0.26351
z_j2 = 0.629864 - 0.62823

# Joint0.0
# R -pi/2 to pi/2
x_j3 = 0.9998 - 0.12393
y_j3 = 0.396488 - 0.31451
z_j3 = 0.657825 - 0.629864

# Palm
# Fixed
# Considering axis of Index finger for palm
x_palm = 0.9997 - 0.9998
y_palm = 0.46651 - 0.396488
z_palm = 0.629955 - 0.657825

x_thumb_base = 0.0999338 - 0.9997
y_thumb_base = 0.524524 - 0.46651
z_thumb_base = 0.612216 - 0.629955

x_index = 0.099 - 0.0999338
y_index = 0.632 - 0.524524
z_index = 0.59 - 0.612216

x_ring = 0.099 - 0.099934
y_ring = 0.632 - 0.632
z_ring = 0.59 - 0.592472

x_thumb = 0.707029 - 0.099
y_thumb = 0.545586 - 0.632
z_thumb = 0.0931454 - 0.59

#Define offsets
x_off = 0.1 #x_j1 + x_gantry_yaw + x_gantry_z + x_gantry_y + x_gantry_x + x_mount + x_tank
y_off = 0.177 #y_j1 + y_gantry_yaw + y_gantry_z + y_gantry_y + y_gantry_x + y_mount + y_tank
z_off = 0.629646 #z_j1 + z_gantry_yaw + z_gantry_z + z_gantry_y + z_gantry_x + z_mount + z_tank

print x_j1 + x_j2 + x_j3 + x_palm + x_thumb_base + x_index + x_off
print y_j1 + y_j2 + y_j3 + y_palm + y_thumb_base + y_index + y_off
print z_j1 + + z_j3 + z_palm + z_thumb_base + z_index + z_off

x_base = x_j1 + x_j2 + x_j3 + x_palm + x_thumb_base
y_base = y_j1 + y_j2 + y_j3 + y_palm + y_thumb_base
z_base = z_j1 + z_j2 + z_j3 + z_palm + z_thumb_base

# Defining all Home positions
M_index = [ 
[1, 0, 0, x_base + x_index ],
[0, 1, 0, y_base + y_index + len_finger ],
[0, 0, 1, z_base + z_index ],
[0,0,0,1]
]

M_ring = [ 
[1, 0, 0, x_base + x_ring ],
[0, 1, 0, y_base + y_ring + len_finger ],
[0, 0, 1, z_base + z_ring ],
[0,0,0,1]
]

M_thumb = [ 
[1, 0, 0, x_base + x_thumb ],
[0, 1, 0, y_base + y_thumb ],
[0, 0, 1, z_base + z_thumb + len_finger],
[0,0,0,1]
]

M_palm = [ 
[1, 0, 0, x_base ],
[0, 1, 0, y_base  + len_palm ],
[0, 0, 1, z_base ],
[0,0,0,1]
]

# Defining Lists of Body Screw Axis
Blist_index = [
[0, 1, 0, -(z_index + z_thumb_base + z_palm + z_j3 + z_j2 + z_j1), 0, -(x_index + x_thumb_base + x_palm + x_j3 + x_j2 + x_j1)],
[1, 0, 0, 0, -(z_index + z_thumb_base + z_palm + z_j3 + z_j2), -(len_finger + y_index + y_thumb_base + y_palm + y_j3 + y_j2)],
[0, 0, 1, -(len_finger + y_index + y_thumb_base + y_palm + y_j3), -(x_index + x_thumb_base + x_palm + x_j3), 0],
[0, 0, 1, len_finger, 0, 0]
]

Blist_ring = [
[0, 1, 0, -(z_ring + z_thumb_base + z_palm + z_j3 + z_j2 + z_j1), 0, -(x_ring + x_thumb_base + x_palm + x_j3 + x_j2 + x_j1)],
[1, 0, 0, 0, -(z_ring + z_thumb_base + z_palm + z_j3 + z_j2), -(len_finger + y_ring + y_thumb_base + y_palm + y_j3 + y_j2)],
[0, 0, 1, -(len_finger + y_ring + y_thumb_base + y_palm + y_j3), -(x_ring + x_thumb_base + x_palm + x_j3), 0],
[0, 0, 1, len_finger, 0, 0]
]

# Defining Lists of Body Screw Axis
Blist_thumb = [
[0, 1, 0, -(len_finger + z_thumb + z_thumb_base + z_palm + z_j3 + z_j2 + z_j1), 0, -(x_thumb + x_thumb_base + x_palm + x_j3 + x_j2 + x_j1)],
[1, 0, 0, 0, -(len_finger + z_thumb + z_thumb_base + z_palm + z_j3 + z_j2), -(y_thumb + y_thumb_base + y_palm + y_j3 + y_j2)],
[0, 0, 1, -(y_thumb + y_thumb_base + y_palm + y_j3), -(x_thumb + x_thumb_base + x_palm + x_j3), 0],
[0, 1, 0, -(len_finger + z_thumb + z_thumb_base), 0, -(x_thumb + x_thumb_base)],
[1, 0, 0, 0, -(len_finger + z_thumb), -(y_thumb)]
]

Blist_palm = [
[0, 1, 0, -(z_palm + z_j3 + z_j2 + z_j1), 0, -(x_palm + x_j3 + x_j2 + x_j1)],
[1, 0, 0, 0, -(z_palm + z_j3 + z_j2), -(len_palm + y_palm + y_j3 + y_j2)],
[0, 0, 1, -(len_palm + y_palm + y_j3), -(x_palm + x_j3), 0]
]

# Forward Kinematics on index finger
def fk_index(thetalist):
    T_old = r.FKinBody(M_index, Blist_index, thetalist)[0:3,3]
    T = np.array([i + j for i, j in zip(T_old, [x_off, y_off, z_off])])
    #print T
    return T

def fk_ring(thetalist):
    T_old = r.FKinBody(M_ring, Blist_ring, thetalist)[0:3,3]
    T = np.array([i + j for i, j in zip(T_old, [x_off, y_off, z_off])])
    #print T
    return T

def fk_thumb(thetalist):
    T_old = r.FKinBody(M_thumb, Blist_thumb, thetalist)[0:3,3]
    T = np.array([i + j for i, j in zip(T_old, [x_off, y_off, z_off])])
    #print T
    return T

def fk_palm(thetalist):
    T_old = r.FKinBody(M_palm, Blist_palm, thetalist)[0:3,3]
    T = np.array([i + j for i, j in zip(T_old, [x_off, y_off, z_off])])
    #print T
    return T

# Inverse Kinematics for index
def ik_index(xg, yg, zg):
    goalpoint = np.array([xg, yg, zg])
    
    T = [[1,0,0,xg - x_off], [0,1,0, yg - y_off], [0,0,1, zg - z_off], [0,0,0,1]]
    print 'To reach'
    print goalpoint

    # gantry_x, gantry_y, gantry_z, gantry_yaw, j1, j2, j3
    thetalist0 =[0.1, 0.1, 0.1, 0.1]
    eomg = 0.01
    ev = 0.001
    [thetalist,success] = r.IKinBody(Blist_index, M_index, T, thetalist0, eomg, ev)
    reachedpoint = fk_index(thetalist)
    #result = np.around(np.linalg.norm(goalpoint - reachedpoint) * 1000) / 1000.0
    print success
    print 'Reached Now'
    print reachedpoint
    return thetalist

# Inverse Kinematics for ring
def ik_ring(xg, yg, zg):
    goalpoint = np.array([xg, yg, zg])
    
    T = [[1,0,0,xg - x_off], [0,1,0, yg - y_off], [0,0,1, zg - z_off], [0,0,0,1]]
    print 'To reach'
    print goalpoint

    # gantry_x, gantry_y, gantry_z, gantry_yaw, j1, j2, j3
    thetalist0 =[0.1, 0.1, 0.1, 0.1]
    eomg = 0.01
    ev = 0.001
    [thetalist,success] = r.IKinBody(Blist_ring, M_ring, T, thetalist0, eomg, ev)
    reachedpoint = fk_ring(thetalist)

    #result = np.around(np.linalg.norm(goalpoint - reachedpoint) * 1000) / 1000.0
    print success
    print 'Reached Now'
    print reachedpoint
    return thetalist

# Inverse Kinematics for thumb
def ik_thumb(xg, yg, zg):
    goalpoint = np.array([xg, yg, zg])
    
    T = [[1,0,0,xg - x_off], [0,1,0, yg - y_off], [0,0,1, zg - z_off], [0,0,0,1]]
    print 'To reach'
    print goalpoint

    # gantry_x, gantry_y, gantry_z, gantry_yaw, j1, j2, j3
    thetalist0 =[0.1, 0.1, 0.1, 0.1, 0.1]
    eomg = 0.01
    ev = 0.001
    [thetalist,success] = r.IKinBody(Blist_thumb, M_thumb, T, thetalist0, eomg, ev)
    reachedpoint = fk_thumb(thetalist)
    #result = np.around(np.linalg.norm(goalpoint - reachedpoint) * 1000) / 1000.0
    print success
    print 'Reached Now'
    print reachedpoint
    return thetalist