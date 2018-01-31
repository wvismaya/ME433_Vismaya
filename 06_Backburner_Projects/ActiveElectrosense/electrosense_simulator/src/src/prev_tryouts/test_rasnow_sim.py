# 3D field as per Rasnow's approximation for spherical Objects
import numpy as np
import math
import sys

def init_params():
	# Frequency of Square wave generated
	# Units: Hz
	global F
	F = 4.7*1000.0

	# Units: mV
	global v1pp
	v1pp = 5.0*1000.0
	global v2pp
	v2pp = 5.0*1000.0

	# Phase difference between the biphasic waves
	global phi
	phi = math.pi

	# Units: cm
	global sphere_radius
	sphere_radius = 2.5

	# Electrical Contrast
	# Perfect Insulators: -1/2
	# PErfect Conductors: +1
	global chi
	chi = -0.5

def v1(t):
	return v1pp*math.cos(2*math.pi*F*t + phi)

def v2(t):
	return v2pp*math.cos(2*math.pi*F*t)

def ef(t,d):
	ex = (v1(t)/d[0,0]) + (v2(t)/d[0,1])
	ey = (v1(t)/d[1,0]) + (v2(t)/d[1,1])
	ez = (v1(t)/d[2,0]) + (v2(t)/d[2,1])
	return [ex, ey, ez]

def target_multiplier(r):
	print [( r[0]/abs( r[0]**3 ) ), ( r[1]/abs( r[1]**3 ) ), ( r[2]/abs( r[2]**3 ) )]
	return [( r[0]/abs( r[0]**3 ) ), ( r[1]/abs( r[1]**3 ) ), ( r[2]/abs( r[2]**3 ) )]

def delta_phi(t, r, d):
	e_total = -ef(t, d)
	r_total = target_multiplier(r)
	return (sphere_radius**3) * chi * ( e_total[0]*r_total[0] + e_total[1]*r_total[1] + e_total[2]*r_total[2] )

def enter_target_loc():
	print('The field boundary is xmin = -1, xmax = 1, ymin = -1, ymax = 1, zmin = -1, zmax = 1')
	s = raw_input('Enter the target location as xyz (Without space or comma): \n')
	return [int(s[0]), int(s[1]), int(s[2])]

# # # # # Main Code starts here:

# # Initialize all the parameters
init_params()

# # Place the hand
# # For now, do not plot. Simply state coordinates

# # Thickness of the hand to act as offset
offset_values = [0.01, 0.1, 0.05]

# # Palm
rx1 = [0., 0., 0.]
# # Ring
tx1 = [5., 0., -2.5]

# # Index
rx2 = [5., 0., 2.5]
# # Thumb
tx2 = [-1.5, 0., 5.]

# # Get position of target
target_loc = [i + j for i, j in zip([0.,0.,0.], offset_values)] #enter_target_loc()

# # Place target in field
# # To be included while plotting

# # Find relative positions
# # # r is Rx wrt centre of target => (Target - Rx)
r1 = [i - j for i, j in zip(target_loc, rx1)]
r2 = [i - j for i, j in zip(target_loc, rx2)]

print [r1,r2]

# # # d is centre of target wrt EACH Tx => (Tx - Target)
d1 = [i - j for i, j in zip(tx1, target_loc)]
d2 = [i - j for i, j in zip(tx2, target_loc)]

d = np.column_stack((d1, d2))
print d

# # Find Delta Phi for location entered
# # When plotting t should be linspace
t = 1.0
print delta_phi(t, r1, d) - delta_phi(t, r2, d)

# # Display the Change in potential occured