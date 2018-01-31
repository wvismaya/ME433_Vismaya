#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import JointState
import robot_calc_functions as r
import numpy as np
from robotdescription import *

def get_angles(data):

	thetalist =[data.position[4], data.position[5], data.position[6], data.position[9]]
	fk_index(thetalist)
	#sleep(1)
	#return T #[0,0,0]

def armbot():
	rospy.init_node('movearmbot')
	rospy.Subscriber('joint_states', JointState, get_angles)

if __name__ == '__main__':
	armbot()
	rospy.spin()