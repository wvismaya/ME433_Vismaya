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
    #print [xg, yg, zg]
    global icangles
    t1, t2, t3 =  fsolve(equations, icangles)
    
    #Get actual angles
    [a1, a2, a3] = [(t1%6.28), (t2%6.28)%-3.14, (t3%6.28)]
    thetalist =[0,0,0]
    T = r.FKinBody(M,Blist,thetalist)[:,3]

    '''

    if( 2.04 < a1):
        a1 = 2.04
    elif(a1 < -2.04):
        a1 = -2.04

    if( 1.57 < a2):
        a2 = 1.57
    elif(a2 < -1.57):
        a2 = -1.57

    if( 1.57 < a3):
        a3 = 1.57
    elif(a3 < -1.57):
        a3 = -1.57

    '''

    result = [a1,a2,a3] #[atan2(sin(a1),cos(a1)),  atan2(sin(a2),cos(a2)),  atan2(sin(a3),cos(a3))]
    #icangles = result
    #print np.round([a1-6.28, a2-6.28, a3-6.28])
    print result 
    return result

def callback(data):
    global xg
    xg = data.x_goal

    global yg
    yg = data.y_goal

    global zg
    zg = data.z_goal
    #rospy.loginfo("%f, %f, %f" % (data.x_goal, data.y_goal, data.z_goal))
    pub = rospy.Publisher('jangles_chatter', Jangles, queue_size=2)
    rate = rospy.Rate(10) #10hz
    msg = Jangles()

    [msg.joint1,msg.joint2,msg.joint3] = get_angles()

    #msg.joint1y= 0.0
    #msg.joint2 = -pi/2
    #msg.joint3 = 0.0
    pub.publish(msg)


def talker():
    rospy.init_node('custom_talker', anonymous=True)
    rospy.Subscriber('path_chatter', PathPts, callback)

if __name__ == '__main__':
    global icangles
    icangles = [0,0,0]
    talker()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()