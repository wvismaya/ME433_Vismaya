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
from testrobot import *

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
my_Vdc = 2000.	# D.C component from generator
circuit_gain = 1. #62.04
## Assume a dipole model
#				palm_rx
#	thumb_rx 				ring_tx
#				index_rx

# Let the noise factor of the apparatus be
# This determines the white noise to be added to the voltage generated.
# More white noise will be added when making measurements
noise_factor_sphere = 0.0

## Measured distances
# Later these values will be read from FK function and Marker Pose. 
# Hence the white noise in later stage too.

# Angle of obj wrt X-Axis in body_frame
#theta_obj = pi
# Distance of obj from origin
#r_obj = 3

## Compute polar position of objects
def get_polar_obj(obj_coord_in):
	## Location of gantry_yaw as a offset
	offset_vals = get_offsets()
	obj_coord = [i - j for i, j in zip(obj_coord_in, offset_vals)]
	return [obj_coord]

## A function to compute polar position of electrode:
def get_polar_electrode(ring_coord_in, index_coord_in, thumb_coord_in, palm_coord_in):
	## Location of gantry_yaw as a offset 
	offset_vals = [0, 0, 0]; #get_offsets()

	ring_coord = [i - j for i, j in zip(ring_coord_in, offset_vals)]
	
	thumb_coord = [i - j for i, j in zip(thumb_coord_in, offset_vals)]
	
	palm_coord = [i - j for i, j in zip(palm_coord_in, offset_vals)]
	
	index_coord = [i - j for i, j in zip(index_coord_in, offset_vals)]
	
	return [ring_coord, thumb_coord, palm_coord, index_coord]

def elc_confg(obj_coord, elec_coords, rxtx):
	global Rx_1
	Rx_1 =  np.linalg.norm(np.array([i - j for i, j in zip(obj_coord, elec_coords[(rxtx+2)%3])]))
	# sqrt( r_obj**2 + rx_palm**2 + 2*r_obj*rx_palm*cos(ang_palm - theta_obj) )
	#print R_palm

	global Rx_2
	Rx_2 = np.linalg.norm(np.array([i - j for i, j in zip(obj_coord, elec_coords[(rxtx+1)%3])]))
	#print R_index

	global Tx_1
	Tx_1 = np.linalg.norm(np.array([i - j for i, j in zip(obj_coord, elec_coords[3])]))
	
	global Tx_2
	Tx_2 = np.linalg.norm(np.array([i - j for i, j in zip(obj_coord, elec_coords[(rxtx)%3])]))
	#sqrt( r_obj**2 + tx_thumb**2 + 2*r_obj*tx_thumb*cos(ang_thumb - theta_obj) )


def get_volts(obj_coord, elec_coords, t, rxtx):
	# rxtx can only take values 0 OR 1
	# The order is: ring_coord, index_coord, thumb_coord, palm_coord
	# Distance of obj wrt electrodes
	## NOTE: Pay attention to extended Pythogorus. Something seems off.
	
	elc_confg(obj_coord, elec_coords, rxtx)
	## Inputs to the systems are voltage and distances of electrodes from object
	my_V_gen_tx1 = my_Vdc*cos( 0. + ((2* pi* (t) ) * 1000.) )
	my_V_gen_tx2 = my_Vdc*cos( pi + ((2* pi* (t) ) * 1000.) )

	## Eccentricity of the object
	my_r = 6.0
	myAR = 1.0001
	my_e = (1 - (1/myAR**2))**0.5

	## Depolarization coefficient
	my_nx = ((1. - my_e**2) / (2 * my_e**3)) * log( (1 + my_e)/(1 - my_e) - 2*my_e )
	
	## Landau Lipschitz term
	# Assume a insulator
	my_M = ( 1. / (my_nx) ) - 1		# The -1 is for insulator
	
	## Volume of object
	my_Vol = 4./3. * pi * myAR * (my_r**3)

	## Electric Dipole moment
	my_P_tx1 = (1.0/4*pi) * (my_V_gen_tx1 / Tx_1) * my_M * my_Vol
	my_P_tx2 = (1.0/4*pi) * (my_V_gen_tx2 / Tx_2) * my_M * my_Vol

	## Reading at each electrode
	phi_rx1 = (( Rx_1 * (my_P_tx1 + my_P_tx2) ) / Rx_1**3) + nr.randn()*noise_factor_sphere
	phi_rx2 = (( Rx_2 * (my_P_tx1 + my_P_tx2) ) / Rx_2**3) + nr.randn()*noise_factor_sphere

	## Voltage obtained in mV
	#print (phi_palm - phi_index) *circuit_gain
	#print (phi_rx1 - phi_rx2)*circuit_gain*my_V_gen_tx2

	#Using the phase sync detector circuit
	return (phi_rx1 - phi_rx2)*circuit_gain**my_V_gen_tx2
