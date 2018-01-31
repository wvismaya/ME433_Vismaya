#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

from hdt_nri_description.msg import PathPts

from math import cos, sin, pi, sqrt, atan2, log
import numpy as np
from scipy.optimize import fsolve
import math
import numpy.random as nr

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
theta_obj = pi
# Distance of obj from origin
r_obj = 3
# Assume the following values of electrodes from origin
rx_dist = 5
tx_dist = 5
# Distance of obj wrt electrodes
## NOTE: Pay attention to extended Pythogorus. Something seems off.
R_palm = sqrt( r_obj**2 + rx_dist**2 - 2*r_obj*rx_dist*cos(theta_obj) )
R_index = sqrt( r_obj**2 + rx_dist**2 - 2*r_obj*rx_dist*cos(pi - theta_obj) )
print R_index
d_ring = sqrt( r_obj**2 + tx_dist**2 - 2*r_obj*tx_dist*cos(1.5708 - theta_obj) )
print d_ring
d_thumb = sqrt( r_obj**2 + tx_dist**2 - 2*r_obj*tx_dist*cos(1.5708 + theta_obj) )

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
print phi_palm - phi_index