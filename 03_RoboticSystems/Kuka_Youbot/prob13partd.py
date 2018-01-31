import numpy as np
import csv
import robot_calc_functions as r
from math import cos, sin, tan, pi
import matplotlib.pyplot as plt


xerrarray = []
thf = []

#PLEASE CHANGE KP KI HERE TO OBSERVE CHANGE IN PLOT AND SIMULATION
kp = 0.9
ki = 0.1

#CODE BEGINS HERE BY DECLARING GLOBAL VARIABLES

xerrarray0 = []
xerrarray1 = []
xerrarray2 = []
xerrarray3 = []
xerrarray4 = []
xerrarray5 = []

Tbo = [
[1,0,0,0],
[0,1,0,0.1662],
[0,0,1,0.0026],
[0,0,0,1]
]

Moe=[
[1, 0, 0, 0],
[0, 1, 0, 0.033],
[0, 0, 1, 0.65],
[0, 0, 0, 1]
]

Blist = [
[0, 0, 1, -0.033, 0,  0],
[1, 0, 0, 0, -0.5076, 0],
[1, 0, 0, 0, -0.3526, 0],
[1, 0, 0, 0, -0.2176, 0],
[0, 0, 1, 0,  0, 0]
]


l = 0.47/2
w = 0.3/2
rd = 0.0475

F = [
[0, 0, 0, 0],
[0, 0, 0, 0],
[(-1.0/l+w)*rd/4, (1.0/l+w)*rd/4, (1.0/l+w)*rd/4, (-1.0/l+w)*rd/4],
[1.0*rd/4, 1.0*rd/4, 1.0*rd/4, 1.0*rd/4 ],
[-1.0*rd/4, 1.0*rd/4, -1.0*rd/4, 1.0*rd/4],
[0, 0, 0, 0]]

def getS(t):
	val = (3/25.0)*t*t - (2.0/125)*t*t*t
	return val

def getX(t):
	s = getS(t)
	X = [ [1, 0,  0, s ], [ 0, sin(s*pi/2),  cos(s*pi/2), 2*s + 1 ], [0, -cos(s*pi/2), sin(s*pi/2), 0.3 + 0.2*s], [0, 0,  0, 1] ]
	return X

def getSdot(t):
	val = (6/25.0)*t - (6.0/125)*t*t
	return val

def getXdot(t):
	s = getS(t)
	sdot = getSdot(t)
	Xdot = [ [0, 0,  0, sdot ], [ 0, sdot*(pi/2)*cos(s*pi/2),  -sdot*(pi/2)*sin(s*pi/2), 2*sdot ], [0, sdot*(pi/2)*sin(s*pi/2), sdot*(pi/2)*cos(s*pi/2), 0.2*sdot], [0, 0,  0, 0] ]
	return Xdot

def getVx(t):
	x = getX(t)
	xdot = getXdot(t)

	vb = r.matmult(r.TransInv(x),xdot)
	return [vb[1,0],vb[2,1],vb[0,2],vb[0,3],vb[1,3],vb[2,3]]

def tse(tlist):
	toe = r.FKinBody(Moe, Blist, tlist[3:])
	tbe = r.matmult(Tbo,toe)
	tsb = [[cos(tlist[0]), -sin(tlist[0]), 0, tlist[1]],[sin(tlist[0]), cos(tlist[0]),  0, tlist[2]],[0, 0, 1, 0.0963],[0, 0, 0, 1]]
	tsemat = r.matmult(tsb,tbe)
	return tsemat

def getxerr(t,tlist):
	x = tse(tlist)
	xd = getX(t)
	xerr = r.MatrixLog6(r.matmult(r.TransInv(x),xd))
	return xerr

def main():
	deltat = 0.01
	thetalist = [0,0,0, 0,0, -pi/2, pi/4, 0]
	#print(getxerr(0.01,thetalist))

	#Initializing error accumulation
	xerrarray.append(getxerr(0, thetalist)*0)
	xerrint = getxerr(0, thetalist)
	tnew = [ thetalist[1], thetalist[2], thetalist[0], thetalist[3], thetalist[4], thetalist[5], thetalist[6], thetalist[7] ]
	thf.append(tnew)

	for i in range(500):
		tym = 0.01*i

		Xd = getX(tym)
		Vd = getVx(tym)

		xerr = np.matrix(getxerr(tym, thetalist))
		#print xerr.shape
		
		xerrarray.append(xerr)

		xerrarray0.append(xerr[0,0])
		xerrarray1.append(xerr[0,1])
		xerrarray2.append(xerr[0,2])
		xerrarray3.append(xerr[0,3])
		xerrarray4.append(xerr[0,4])
		xerrarray5.append(xerr[0,5])
		
		#print np.matrix(xerrarray[len(xerrarray)-1]).shape
		xerrint = np.add(xerrint, getxerr(tym, thetalist))*0.01
		
		x = tse(thetalist)
		
		ad = r.Adjoint(r.matmult(r.TransInv(x),Xd))
		vt = r.matmult(ad,Vd) + kp*np.asarray(getxerr(tym, thetalist)) + ki*np.asarray(xerrint)
		
		toe = r.FKinBody(Moe, Blist, thetalist[3:])
		tsb = [[cos(thetalist[0]), -sin(thetalist[0]), 0, thetalist[1]],
		[sin(thetalist[0]), cos(thetalist[0]),  0, thetalist[2]],
		[0, 0, 1, 0.0963],
		[0, 0, 0, 1]]
		
		jarm = r.JacobianBody(Blist, thetalist[3:])
		jbase = r.matmult(r.Adjoint(r.matmult(r.TransInv(toe),r.TransInv(Tbo))),F)
		je = np.concatenate((jbase, jarm),axis=1)
		
		uthed = r.matmult(np.linalg.pinv(je),vt)
		u = uthed[:4]
		vb6 = tym*r.matmult(F,u)[2:5]
		
		thetader = 0.01*np.concatenate((vb6,uthed[4:]),axis=1)
		thetalist = np.add(thetalist, thetader)

		tnew = [ thetalist[1], thetalist[2], thetalist[0], thetalist[3], thetalist[4], thetalist[5], thetalist[6], thetalist[7] ]
		
		thf.append(tnew)
		#print np.matrix(Xd)

		#print getxerr(5,thf[len(thf)-1])

	i = range(0,500)
	plt.plot(i,xerrarray0)
	plt.plot(i,xerrarray1)
	plt.plot(i,xerrarray2)

	plt.plot(i,xerrarray3)
	plt.plot(i,xerrarray4)
	plt.plot(i,xerrarray5)

	plt.show()

if __name__ == '__main__':
	main()
	print len(thf)
	with open("outputPartD.csv", "wb") as f:
		writer = csv.writer(f)
		writer.writerows(thf)