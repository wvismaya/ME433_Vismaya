#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

from hdt_nri_description.msg import PathPts
from sensor_msgs.msg import JointState

from math import cos, sin, pi, sqrt, atan2, log
import numpy as np
from scipy.optimize import fsolve
import math
import numpy.random as nr
from robotdescription import *

from random import random

# To generate electric field we would need the following:
# myVol Voltage of square wave
# R_palm Distance of palm electrode from object
# R_index Distance of index finger electrode from object
# d_ring Distance of ring finger electrode from object
# d_thumb Distance of thumb electrode from object
# myAR Aspect ratio of spheriod
# myr semi-minor axis of object

# NOTE: All measurements in m

## Inputs to the systems are voltage and distances of electrodes from object
my_V_gen = 20
## Assume a dipole model
#				palm_rx
#	thumb_rx 				ring_tx
#				index_rx

# Let the noise factor of the apparatus be
# This determines the white noise to be added to the voltage generated.
# More white noise will be added when making measurements
noise_factor_sphere = 0.0003

## Measured distances
# Later these values will be read from FK function and Marker Pose. 
# Hence the white noise in later stage too.

# Angle of obj wrt X-Axis in body_frame
#theta_obj = pi
# Distance of obj from origin
#r_obj = 3
# Assume the following values of electrodes from origin
ring_coord = [0, 0, 5]
tx_ring = sqrt(ring_coord[0]*2 + ring_coord[1]**2 + ring_coord[2]**2)
ang_ring = atan2(ring_coord[2], sqrt(ring_coord[0]**2 + ring_coord[1]**2))

thumb_coord = [0, 0, -5]
tx_thumb = sqrt(thumb_coord[0]*2 + thumb_coord[1]**2 + thumb_coord[2]**2)
ang_thumb = atan2(thumb_coord[2], sqrt(thumb_coord[0]**2 + thumb_coord[1]**2))

palm_coord = [2.2, 2.2, 0]
rx_palm = sqrt(palm_coord[0]*2 + palm_coord[1]**2 + palm_coord[2]**2)
ang_palm = atan2(palm_coord[2], sqrt(palm_coord[0]**2 + palm_coord[1]**2))

index_coord = [-2.2, 2.2, 0]
rx_index = sqrt(index_coord[0]*2 + index_coord[1]**2 + index_coord[2]**2)
ang_index = atan2(index_coord[2], sqrt(index_coord[0]**2 + index_coord[1]**2))
## A function to compute polar position of electrode:
def get_polar_electrode(ring_coord, index_coord, thumb_coord, palm_coord):
	ring_coord = [0, 0, 5]
	tx_ring = sqrt(ring_coord[0]*2 + ring_coord[1]**2 + ring_coord[2]**2)
	ang_ring = atan2(ring_coord[2], sqrt(ring_coord[0]**2 + ring_coord[1]**2))

	thumb_coord = [0, 0, -5]
	tx_thumb = sqrt(thumb_coord[0]*2 + thumb_coord[1]**2 + thumb_coord[2]**2)
	ang_thumb = atan2(thumb_coord[2], sqrt(thumb_coord[0]**2 + thumb_coord[1]**2))

	palm_coord = [2.2, 2.2, 0]
	rx_palm = sqrt(palm_coord[0]*2 + palm_coord[1]**2 + palm_coord[2]**2)
	ang_palm = atan2(palm_coord[2], sqrt(palm_coord[0]**2 + palm_coord[1]**2))

	index_coord = [-2.2, 2.2, 0]
	rx_index = sqrt(index_coord[0]*2 + index_coord[1]**2 + index_coord[2]**2)
	ang_index = atan2(index_coord[2], sqrt(index_coord[0]**2 + index_coord[1]**2))

	return [tx_ring, ang_ring, tx_thumb, ang_thumb, ]

def get_volts(r_obj, theta_obj):
	# Distance of obj wrt electrodes
	## NOTE: Pay attention to extended Pythogorus. Something seems off.
	R_palm = sqrt( r_obj**2 + rx_palm**2 - 2*r_obj*rx_palm*cos(ang_palm - theta_obj) )
	R_index = sqrt( r_obj**2 + rx_index**2 - 2*r_obj*rx_index*cos(ang_index - theta_obj) )
	d_ring = sqrt( r_obj**2 + tx_ring**2 - 2*r_obj*tx_ring*cos(ang_ring - theta_obj) )
	d_thumb = sqrt( r_obj**2 + tx_thumb**2 - 2*r_obj*tx_thumb*cos(ang_thumb - theta_obj) )

	## Eccentricity of the object
	my_r = 1
	myAR = 1.0001
	my_e = (1 - (1/myAR**2))**0.5

	## Depolarization coefficient
	my_nx = ((1 - my_e**2) / (2 * my_e**3)) * log( (1 + my_e)/(1 - my_e) - 2*my_e )
	
	## Landau Lipschitz term
	# Assume a insulator
	my_M = 1 / (my_nx - 1)		# The -1 goes away if a conductor
	
	## Volume of object
	my_Vol = 4/3 * pi * myAR * (my_r**3)

	## Electric Dipole moment
	my_P_ring = (1.0/4*pi) * (my_V_gen / d_ring) * my_M * my_Vol
	my_P_thumb = (1.0/4*pi) * (my_V_gen / d_thumb) * my_M * my_Vol

	## Reading at each electrode
	phi_palm = (( R_palm * (my_P_ring + my_P_thumb) ) / R_palm**3) + nr.randn()*noise_factor_sphere
	phi_index = (( R_index * (my_P_ring + my_P_thumb) ) / R_index**3) + nr.randn()*noise_factor_sphere

	## Voltage obtained in mV
	return phi_palm - phi_index

def callback(data):
	new_r_obj = sqrt(data.x_goal**2 + data.y_goal**2 + data.z_goal**2)
	new_theta_obj = atan2(data.z_goal, sqrt(data.x_goal**2 + data.y_goal**2))
	# Call adc gen function and get values of voltages
	print get_volts(new_r_obj, new_theta_obj)
	#rospy.loginfo("%f, %f, %f" % (data.x_goal, data.y_goal, data.z_goal))

    
def talker():
    rospy.init_node('generate_adc', anonymous=True)
    rospy.Subscriber('path_chatter', PathPts, callback)


if __name__ == '__main__':
    talker()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()