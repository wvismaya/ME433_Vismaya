#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

from hdt_nri_description.msg import PathPts, myADC
from sensor_msgs.msg import JointState

from math import cos, sin, pi, sqrt, atan2, log
import numpy as np
import numpy.random as nr
from testrobot import *
from adcgen_lib import *

import random

box_bound = 1.
bound_x = box_bound
bound_y = box_bound
bound_z = box_bound

def get_loc():
	global now_new
	now_new = 0.5 #rospy.Time.now().secs - now_first
	get_confg()

	global delta_V
	delta_V.diff_voltage_value = get_volts(obj_coord, [ring_coord, index_coord, thumb_coord, palm_coord], now_new, 0)
	
	#print delta_V.diff_voltage_value
	#rospy.loginfo(msg)
	pub.publish(delta_V)

def get_confg():
	x_loc = 1.
	y_loc = 1.
	z_loc = 1.
	#print 'ring'
	#print get_volts(obj_coord, [ring_coord, index_coord, thumb_coord, palm_coord], now_new, 0)

	#print 'index'
	#print get_volts(obj_coord, [ring_coord, index_coord, thumb_coord, palm_coord], now_new, 1)
	
	#print 'thumb'
	#print get_volts(obj_coord, [ring_coord, index_coord, thumb_coord, palm_coord], now_new, 2)
	expected_phi = get_volts(obj_coord, [ring_coord, index_coord, thumb_coord, palm_coord], now_new, 0)
	grid_var = 30.
	grid_pts = 3000
	ii = 1.
	w_old = 99.

	for ii in range(3):
		for n in range(grid_pts):
			xx = random.uniform(-1, 1)
			yy = random.uniform(-1, 1)
			zz = random.uniform(-1, 1)
			w = abs(expected_phi - get_volts([xx, yy, zz], [ring_coord, index_coord, thumb_coord, palm_coord], now_new, 0))
			if(w <= grid_var):
				if(w<=1000.):
					print('\n')
					print('Estimation is:')
					print([xx, yy, zz])
					print('\n')
					w_old = w
					print(w)
					x_loc = xx
					y_loc = yy
					z_loc = zz

def save_fingers(data):
	thetalist_index =[data.position[4], data.position[5], data.position[6], data.position[9]]
	thetalist_ring =[data.position[4], data.position[5], data.position[6], data.position[10]]
	thetalist_thumb =[data.position[4], data.position[5], data.position[6], data.position[7], data.position[8]]
	thetalist_palm =[data.position[4], data.position[5], data.position[6]]

	global index_coord
	#print np.linalg.norm(np.array(index_coord))
	index_coord = fk_index(thetalist_index)
	
	global ring_coord
	#print ring_coord
	ring_coord = fk_ring(thetalist_ring)

	global thumb_coord
	#print thumb_coord
	thumb_coord = fk_thumb(thetalist_thumb)

	global palm_coord
	#print np.linalg.norm(np.array(palm_coord))
	palm_coord = fk_palm(thetalist_palm)

	now = rospy.Time.now().secs - now_first

	#print 'Estimated Location is:'
	get_loc()
	#print finger_details

def save_obj(data):
	global obj_coord
	obj_coord = [data.x_goal, data.y_goal, data.z_goal]
	print 'Object located at:'
	print obj_coord

	now = rospy.Time.now().secs - now_first #rospy.get_rostime()

	get_loc()

	# global delta_V
	# delta_V.diff_voltage_value = get_volts(obj_coord, [ring_coord, index_coord, thumb_coord, palm_coord], now, 0)

	# print delta_V.diff_voltage_value
	# #rospy.loginfo(msg)
	# pub.publish(delta_V)
	# print '\n'

def listener():
	global pub
	pub = rospy.Publisher('adc_chatter', myADC, queue_size=10)
	rospy.Subscriber('path_chatter', PathPts, save_obj)
	rospy.Subscriber('joint_states', JointState, save_fingers)
	rospy.spin()

if __name__ == '__main__':
	global delta_V
	delta_V = myADC()
	global obj_coord
	obj_coord = [100, 100, 100]
	global index_coord
	index_coord = fk_index([0, 0, 0, 0])
	global ring_coord
	ring_coord = fk_ring([0, 0, 0, 0])
	global thumb_coord
	thumb_coord = fk_thumb([0, 0, 0, 0, 0])
	global palm_coord
	palm_coord = fk_palm([0, 0, 0])
	
	rospy.init_node('test_node', anonymous=True)
	global now_first
	now_first = rospy.Time.now().secs #rospy.get_rostime()
	print get_volts([-1,-1,-1], [ring_coord, index_coord, thumb_coord, palm_coord], 0.0005, 0)
	print get_volts([1,1,1], [ring_coord, index_coord, thumb_coord, palm_coord], 0.0005, 0)
	listener()