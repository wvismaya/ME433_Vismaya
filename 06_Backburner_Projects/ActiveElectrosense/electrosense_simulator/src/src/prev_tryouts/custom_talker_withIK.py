#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2, pi
import hdt_nri_description.msg as hnd
import robot_calc_functions as r
import random

#Define offsets
x_off = 0.5
y_off = -0.2
z_off = -0.2

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

# Joint1
# R -2.094 to 2.094
x_j1 = 0.0
y_j1 = -0.068
z_j1 = 0.0

# Joint1_FIXED
# Fixed
x_j1f = 0.0
y_j1f = -0.0865
z_j1f = 0.0

# Joint2
# R -pi/2 to pi/2
x_j2 = 0.02393
y_j2 = -0.0510
z_j2 = 0.0

# Joint0.0
# R -pi/2 to pi/2
x_j3 = -0.02395
y_j3 = -0.08200
z_j3 = -0.0279

# Palm
# Fixed
# Considering axis of Index finger for palm
x_palm = 0.0
y_palm = -0.07
z_palm = 0.028

x_index = -0.0000362
y_index = -0.165998
z_index = 0.0357953

x_ring = -0.0000362
y_ring = -0.165998
z_ring = -0.0182153

x_m = x_palm + x_j3 + x_j2 + x_j1f  + x_j1 + x_gantry_yaw + x_gantry_z + x_gantry_y + x_gantry_x + x_mount
y_m = y_palm + y_j3 + y_j2 + y_j1f + y_j1 + y_gantry_yaw + y_gantry_z + y_gantry_y + y_gantry_x + y_mount
z_m = z_palm + z_j3 + z_j2 + z_j1f + z_j1 + z_gantry_yaw + z_gantry_z + z_gantry_y + z_gantry_x + z_mount

M_index = [ [1, 0, 0, x_m + x_index ], [0, 1, 0, y_m + y_index], [0, 0, 1, z_m + z_index ], [0,0,0,1]]
M_ring = [ [1, 0, 0, x_m + x_ring ], [0, 1, 0, y_m + y_ring], [0, 0, 1, z_m + z_ring ], [0,0,0,1]]

Blist_index = [

[0, 1, 0, -(z_index + z_palm + z_j3 + z_j2 + z_j1f), 0, -(x_index + x_palm + x_j3 + x_j2 + x_j1f)], 
[1, 0, 0, 0, -(z_index + z_palm + z_j3), -(y_index + y_palm + y_j3)], 
[0, 0, 1, -(y_index + y_palm), -(x_index + x_palm), 0],
[0, 0, 1, 0, 0, 0]
]

Blist_ring = [

[0, 1, 0, -(z_ring + z_palm + z_j3 + z_j2 + z_j1f), 0, -(x_ring + x_palm + x_j3 + x_j2 + x_j1f)], 
[1, 0, 0, 0, -(z_ring + z_palm + z_j3), -(y_ring + y_palm + y_j3)], 
[0, 0, 1, -(y_ring + y_palm), -(x_ring + x_palm), 0],
[0, 0, 1, 0, 0, 0]
]

maxtime = 3

def set_inlimit(tlist):
    # For the Joint1 2pi/3 to -2pi/3
    if( np.sign(tlist[0]) == 1):
        tlist[0] = tlist[0]%6.28

    elif( np.sign(tlist[0]) == -1):
        tlist[0] = tlist[0]%-6.28

    if( tlist[0] > 2.094 or tlist[0] < -2.094 ):
        tlist[0] = 0

    # For the Joint2 pi/2 to -pi/2

    if( np.sign(tlist[1]) == 1):
        tlist[1] = tlist[1]%6.28

    elif( np.sign(tlist[1]) == -1):
        tlist[1] = tlist[1]%-6.28

    if( tlist[1] > 1.57 or tlist[1] < -1.57 ):
        tlist[1] = 0

    # For the Joint3 pi/2 to -pi/2

    if( np.sign(tlist[2]) == 1):
        tlist[2] = tlist[2]%6.28

    elif( np.sign(tlist[2]) == -1):
        tlist[2] = tlist[2]%-6.28

    if( tlist[2] > 1.57 or tlist[2] < -1.57 ):
        tlist[2] = 0

    # For the Fingers 0 to pi/2

    if( np.sign(tlist[3]) == 1):
        tlist[3] = tlist[3]%6.28

    elif( np.sign(tlist[3]) == -1):
        tlist[3] = tlist[3]%-6.28

    if( tlist[3] > 1.57 or tlist[3] < 0 ):
        tlist[3] = 0

    return tlist


def get_angles(blist, mhome):
    Blist = blist
    M = mhome

    i = 0
    oldresult = 10
    goalpoint = np.array([xg,yg,zg,1])
    
    T = [[1,0,0,xg], [0,1,0,yg], [0,0,1,zg], [0,0,0,1]]

    # gantry_x, gantry_y, gantry_z, gantry_yaw, j1, j2, j3
    thetalist0 =[0,0,0, 0]
    eomg = 0.01
    ev = 0.01
    [thetalist,success] = r.IKinBody2(Blist, M, T, thetalist0, eomg, ev)
    reachedpoint = r.FKinBody(M,Blist,thetalist)[:,3]
    result = np.around(np.linalg.norm(goalpoint - reachedpoint) * 1000) / 1000.0
    thetasum = thetalist

    while(i < maxtime):
        #print 'Calculating IK'
        thetalist0 = thetalist #[random.uniform(-2.04, 2.04), random.uniform(-1.57, 1.57), random.uniform(-1.57, 1.57)]
        [thetalist,success] = r.IKinBody2(Blist, M, T, thetalist0, eomg, ev)
        reachedpoint = r.FKinBody(M,Blist,thetalist)[:,3]
        result = np.around(np.linalg.norm(goalpoint - reachedpoint) * 1000) / 1000.0
        #if(oldresult > result):
        #    oldresult = result
        #    thetaresult = thetalist
        thetasum = np.add(thetasum, thetalist)
        thetaresult = thetasum/(maxtime+1)

        i += 1

    #print 'averaged angles'
    #print thetasum/(maxtime+1)

    #print 'Error in each direction'
    #print np.matrix([xg,yg,zg,1]) - reachedpoint

    print 'Obtained position'
    print reachedpoint
    print 'Required position'
    print np.matrix([xg,yg,zg,1])

    #print 'Error value'
    #print result    
    print '\n'
    #print success
    return thetaresult

def callback(data):
    global xg
    xg = data.x_goal #+ x_off

    global yg
    yg = data.y_goal #+ y_off

    global zg
    zg = data.z_goal - 0.44 + z_off
    
    #rospy.loginfo("%f, %f, %f" % (data.x_goal, data.y_goal, data.z_goal))
    pub = rospy.Publisher('jangles_chatter', hnd.Jangles, queue_size=1)
    #rate = rospy.Rate(10) #10hz
    msg1 = hnd.Jangles()

    indexangles = set_inlimit(get_angles(Blist_index, M_index))
    ringangles = set_inlimit(get_angles(Blist_ring, M_ring))

    myjointangles = [0.5*(indexangles[0]+ringangles[0]),  -0.5*(indexangles[1]+ringangles[1]),  0.5*(indexangles[2]+ringangles[2]), indexangles[3], ringangles[3]]

    #print np.matrix(myjointangles)
    #print '\n'

    [msg1.gantry_x, msg1.gantry_y, msg1.gantry_yaw] = [0, 0, pi]
    [msg1.joint1, msg1.joint2, msg1.joint3, msg1.index_joint, msg1.ring_joint] = myjointangles #get_angles(Blist_index)
    pub.publish(msg1)
    # thetas = get_angles()
    # msg.joint1= thetas[4]
    # msg.joint2 = thetas[5]
    # msg.joint3 = thetas[6]
    
def talker():
    rospy.init_node('custom_talker', anonymous=True)
    rospy.Subscriber('path_chatter', hnd.PathPts, callback)


if __name__ == '__main__':
    talker()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()