#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import hdt_nri_description.msg as hnd
import robot_calc_functions as r
from testrobot import *
from math import pi

def callback_ik(data):
    global xg
    xg = data.x_goal #- x_off

    global yg
    yg = data.y_goal #- y_off

    global zg
    zg = data.z_goal #- z_off

    # mean_finger_pose = np.array([(i + j)/2 for i, j in zip(reached_here_ring, reached_here_index)])
    # print 'Mean pose'
    # print np.array([i - j for i, j in zip(mean_finger_pose, [xg, yg, zg])])

    new_thetalist_thumb = ik_thumb(xg, yg, zg)
    print new_thetalist_thumb

    print 'Solving IK'
    new_thetalist = np.array([(i + j)/2 for i, j in zip(ik_index(xg, yg, zg), ik_ring(xg, yg, zg))])
    print new_thetalist
    #print reached_here
    #print ik_index(xg, yg, zg)
    print '\n'

    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    joint_state = JointState()

    joint_state.header.stamp = rospy.Time.now()
    t = rospy.get_time()
    joint_state.name = ['gantryX','gantryY','gantryZ','gantryYaw','joint1','joint2','joint3','thumb_base','thumb_prox','index_prox','ring_prox']

    jangs = [0, 0, 0, 0, 
    new_thetalist[0],
    new_thetalist[1], 
    new_thetalist[2],
    -new_thetalist_thumb[3],
    new_thetalist_thumb[4],
    -new_thetalist[3],
    -new_thetalist[3]]
    #Fingers are zero to pi/2, arm is -pi/2 to pi/2

    print jangs

    joint_state.position = jangs #[0,0,0,0, jangs[0],jangs[1],jangs[2], 0, 0, 0, 0] # update joint_states with time
    joint_pub.publish(joint_state)

def callback_fk(data):
    thetalist_index =[data.position[4], data.position[5], data.position[6], data.position[9]]
    thetalist_ring =[data.position[4], data.position[5], data.position[6], data.position[10]]
    #global reached_here_index
    #reached_here_index = fk_index(thetalist_index)

    #global reached_here_ring
    #reached_here_ring = fk_ring(thetalist_ring)
    
def talker():
    rospy.init_node('ik_fk_tester', anonymous=True)
    rospy.Subscriber('path_chatter', hnd.PathPts, callback_ik)
    #rospy.Subscriber('joint_states', JointState, callback_fk)

if __name__ == '__main__':
    talker()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()