#!/usr/bin/env python
import rospy
from scipy.optimize import fsolve
from sympy import *
import numpy as np
from math import atan2, pi
from hdt_nri_description.msg import Jangles, PathPts
import robot_calc_functions as r

#Define lengths from elbow to palm

x1 = 0;
y1 = -0.068;
z1 = 0;

x2 = 0.02393;
y2 = -0.0510 - 0.08651;
z2 = 0;

x3 = -0.02395;
y3 = 0.08200;
z3 = -0.0279;

x4 = -0.00001;
y4 = -0.07;
z4 = 0.02793;

xg = 0.0086
yg = -0.197
zg = 0.0187

M = [[1, 0, 0, x1 + x2 + x3 + x4], [0, 1, 0, y1 + y2 + y3 + y4], [0, 0, 1, z1 + z2 + z3 + z4], [0,0,0,1]]
Blist = [[0, 1, 0, 0, 0, 0],[1, 0, 0, 0, z4 + z3, y4 + y3],[0, 0, 1, y4, x4, 0]]

def equations(p):
    t1, t2, t3 = p
    xeq = 0.012*sin(t1)*sin(t2)-0.07*cos(t1)*sin(t3)-0.07*sin(t1)*sin(t2)*(1 - cos(t3)) - xg
    yeq = -0.1935 - 0.012*(1 - cos(t2)) - 0.07*cos(t2)*(1 - cos(t3)) - yg
    zeq = 0.012*cos(t1)*sin(t2) + 0.07*sin(t1)*sin(t3) -  0.07*cos(t1)*sin(t2)*(1 - cos(t3)) - zg
    #print nsolve(xeq,yeq,zeq,(t1,t2,t3),(0,0,0))
    return (xeq,yeq,zeq)

def get_angles():
    goalpoint = np.array([xg,yg,zg,1])
    T = [[0,1,0,xg], [0,0,-1,yg], [-1,0,0,zg], [0,0,0,1]]
    thetalist0 =[0,0,0]
    eomg = 0.01
    ev = 0.001
    [thetalist,success] = r.IKinBody2(Blist, M, T, thetalist0, eomg, ev)
    reachedpoint = r.FKinBody(M,Blist,thetalist)[:,3]
    result = np.around(np.linalg.norm(goalpoint - reachedpoint) * 1000) / 1000.0
    #print success
    return reachedpoint

def callback(data):
    global xg
    xg = data.x_goal

    global yg
    yg = data.y_goal

    global zg
    zg = data.z_goal

    print np.matrix(get_angles())
    print np.matrix([xg,yg,zg,1])
    '''
    #rospy.loginfo("%f, %f, %f" % (data.x_goal, data.y_goal, data.z_goal))
    pub = rospy.Publisher('jangles_chatter', Jangles, queue_size=2)
    rate = rospy.Rate(10) #10hz
    msg = Jangles()

    [msg.joint1,msg.joint2,msg.joint3] = get_angles()

    #msg.joint1y= 0.0
    #msg.joint2 = -pi/2
    #msg.joint3 = 0.0
    pub.publish(msg)
    '''


def talker():
    rospy.init_node('custom_talker', anonymous=True)
    rospy.Subscriber('path_chatter', PathPts, callback)


if __name__ == '__main__':
    global icangles
    icangles = [0,0,0]
    talker()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()