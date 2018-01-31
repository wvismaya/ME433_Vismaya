# Algorithm
# For each particle m = 1 to M, do
#	Assume m is the true location
#	for each control option c = 1 to 16, do
#		Simulate control option c and find the variance of resultant particles
#		based on expected observation
#	end for
#	Particle m votes for control action with least variance
# end for
# Choose control with most votes

import math as mt
import random
import numpy as np
import matplotlib.pyplot as plt

d = 10

# This will be the case when writing 2D slice of water in front of the hand.
# The issue for exploring in such case is the slice of hand changes planes.
# So close the thumb and try again: so now it aligns with the finger direction

tx1 = [ 5., 0, 0]
tx2 = [-5., 0, 0]

rx1 = [0,  5., 0]
rx2 = [0, -5., 0]

def get_dist(x1, y1, z1, x2, y2, z2):
	return mt.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) + (z1 - z2) ** 2 + 0.001

def Ef(x,y,z):
	t = 0.0005
	Vdc = 2000
	# Frequency is 1000Hz
	e1 = Vdc*mt.cos(2*mt.pi*t*1000)/get_dist(tx1[0], tx1[1], tx1[2], x, y, z)
	e2 = Vdc*mt.cos(2*mt.pi*t*1000 + mt.pi)/ get_dist(tx2[0], tx2[1], tx2[2], x, y, z)
	return e1+e2

def get_r(x, y, z):
	r1 = get_dist(rx1[0], rx1[1], rx1[2], x, y, z)
	r2 = get_dist(rx2[0], rx2[1], rx2[2], x, y, z)
	return ((r1**-1) - (r2**-1))

def dPhi(x,y, z):
	a = 3.8
	chi = 1.
	return (a**3)*chi*Ef(x,y,z)*get_r(x,y,z)

# Loop over entire grid to get phi
x_grid = d
y_grid = d
z_grid = d

level = 0.002

noise_level = 20
noise = random.uniform(-noise_level, noise_level)

x_exp = random.uniform(-d, d)
y_exp = random.uniform(-d, d)
z_exp = random.uniform(-d, d)
print([x_exp, y_exp, 0])
print('\n')

expected_value = dPhi(x_exp, y_exp, 0) + noise

possible_valuesx = []
possible_valuesy = []
possible_valuesz = []

phi_var = 0
i = 3
for xx in np.linspace(-x_grid, x_grid, 3000):
	for yy in np.linspace(-y_grid, y_grid, 3000):
		zz = 0
		# for zz in np.linspace(-z_grid, z_grid, 3000):
		# 	if (abs(dPhi(xx,yy,zz) - expected_value)) < 0.0002:
		# 		print( [xx, yy])
		# 		possible_valuesx.append(xx)
		# 		possible_valuesy.append(yy)
		# 		print('\n')
		# 		i += 1
			# let sigma be 0.001
		# possible_valuesx.append(xx)
		# possible_valuesy.append(yy)
		# possible_valuesz.append(zz)
		# print( [xx, yy])
		# print([dPhi(xx,yy,zz), expected_value])
		# print(abs(dPhi(xx,yy,zz) - expected_value))
		phi_var = (1/(i-1))*( (i-2)*phi_var + ( (i-1)/i) * ( (dPhi(xx, yy, zz) - expected_value)**2) )
		if (abs(dPhi(xx,yy,zz) - expected_value)) < 0.0002:
			print( [xx, yy])
			possible_valuesx.append(xx)
			possible_valuesy.append(yy)
			print('\n')
		i += 1

print('\n')

plt.scatter(x_exp, y_exp, color='r')
plt.scatter(possible_valuesx, possible_valuesy, color='c')
plt.show()
# tx1 = [3., 0]
# rx1 = [0 , 3.]

# expected_value = dPhi(x_exp, y_exp) + noise

# for xx in np.linspace(-x_grid, x_grid, 3000):
# 	for yy in np.linspace(-y_grid, y_grid, 3000):
# 		# let sigma be 0.001
# 		if abs(dPhi(xx,yy) - expected_value) <= level:
# 			print([xx, yy])
# 		#print( [xx, yy])
# 		#print([dPhi(xx,yy), expected_value])
# 		#print(abs(dPhi(xx,yy) - expected_value))

# print('\n')