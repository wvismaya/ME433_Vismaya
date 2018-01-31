#!/usr/bin/env python
import rospy
from hdt_nri_description.msg import PathPts
from math import cos, sin, pi, sqrt, atan2
import robot_calc_functions as r
import numpy as np
from scipy.optimize import fsolve
import math

T = 10

def path_points():

    r = math.sqrt(0.0267941)
    x0 = -0.02395
    y0 = 0.082
    z0 = -0.0279

    global x
    x = []
    global y
    y = []
    global z
    z = []

    for i in range(0,100,7):
        for j in range(0,100,7):
            x.append(x0 + r*math.cos(i/100.0)*math.cos(j/100.0))
            y.append(y0 + r*math.cos(i/100.0)*math.sin(j/100.0))
            z.append(z0 + r*math.sin(i/100.0))
    #return [x,y,z]

def get_points(tym):
    return [x[tym%len(x)],y[tym%len(y)],z[tym%len(z)]]

def talker():
    pub = rospy.Publisher('path_chatter', PathPts, queue_size=10)
    rate = rospy.Rate(5) #5hz
    
    
    #print t_new
    msg = PathPts()

    #'''
    xg = 0
    yg = -0.2055
    zg = 0.012
    '''
    xg = 0.0086
    yg = -0.197
    zg = -0.0187
    '''

    #[msg.x_goal, msg.y_goal, msg.z_goal] = [xg,yg,zg] #get_points(t_new)

    while not rospy.is_shutdown():
        t_new = rospy.get_rostime()-t_old
        rospy.loginfo("Current time %i %i", t_new.secs, t_new.nsecs)

        [msg.x_goal, msg.y_goal, msg.z_goal] = [xg, yg, zg] #get_points(t_new.secs)
        rospy.loginfo(msg)
        
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('path_talker', anonymous=True)
    t_old = rospy.get_rostime()
    rospy.loginfo("Current time %i", t_old.secs)
    path_points()

    try:
        talker()
    except rospy.ROSInterruptException: pass