#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2, pi
import hdt_nri_description.msg as hnd
import robot_calc_functions as r
import random
from robotdescription import *

def callback(data):
    global xg
    xg = data.x_goal #- x_off

    global yg
    yg = data.y_goal #- y_off

    global zg
    zg = data.z_goal #- z_off
    
    #rospy.loginfo("%f, %f, %f" % (data.x_goal, data.y_goal, data.z_goal))
    #pub = rospy.Publisher('jangles_chatter', hnd.Jangles, queue_size=1)
    #rate = rospy.Rate(10) #10hz
    #msg1 = hnd.Jangles()

    # indexangles = set_inlimit(get_angles_finger(Blist_index, M_index, xg, yg, zg))
    # ringangles = set_inlimit(get_angles_finger(Blist_ring, M_ring, xg, yg, zg))
    # thumbangles = set_inlimit(get_angles_thumb(Blist_thumb, M_thumb, xg, yg, zg))

    indexangles = ik_index(xg, yg, zg)
    #ringangles = get_angles_finger(Blist_ring, M_ring, xg, yg, zg)
    #thumbangles = get_angles_thumb(Blist_thumb, M_thumb, xg, yg, zg)

    #myjointangles = [0.33*(thumbangles[0]+indexangles[0]+ringangles[0]),  -0.33*(thumbangles[1]+indexangles[1]+ringangles[1]),  -0.33*(thumbangles[2]+indexangles[2]+ringangles[2]), thumbangles[3], 0, 0.5*(indexangles[3]+ringangles[3]), 0.5*(indexangles[3]+ringangles[3])]
    #myjointangles = [indexangles[0],  indexangles[1]%6.28,  indexangles[2]%6.28, 0, 0, indexangles[3]%6.28, 0]
    #print np.matrix(myjointangles)
    #print '\n'

    #[msg1.gantry_x, msg1.gantry_y, msg1.gantry_yaw] = [0, 0, pi]
    #[msg1.joint1, msg1.joint2, msg1.joint3, msg1.thumb_base, msg1.thumb_joint, msg1.index_joint, msg1.ring_joint] = myjointangles #get_angles(Blist_index)
    #pub.publish(msg1)
    
def talker():
    rospy.init_node('custom_talker', anonymous=True)
    rospy.Subscriber('path_chatter', hnd.PathPts, callback)


if __name__ == '__main__':
    talker()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()