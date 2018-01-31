import numpy as np
import robot_calc_functions as r
from math import cos, sin, tan, pi
import csv

Blist = [
[0, 0, 1, -0.033, 0,  0],
[1, 0, 0, 0, -0.5076, 0],
[1, 0, 0, 0, -0.3526, 0],
[1, 0, 0, 0, -0.2176, 0],
[0, 0, 1, 0,  0, 0]
]

Tbo = [
[1,0,0,0],
[0,1,0,0.1662],
[0,0,1,0.0026],
[0,0,0,1]
]

X = [
[1, 0,  0, 0],
[0, 0,  1, 1],
[0, -1, 0, 0.4],
[0, 0,  0, 1]
]

Moe=[
[1, 0, 0, 0],
[0, 1, 0, 0.033],
[0, 0, 1, 0.65],
[0, 0, 0, 1]
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

thetalist0 =[0.001,0.001,0.001, 0.001,0.001, -pi/2, 0.001,0.001]
eomg = 0.001
ev = 0.01


def hw5_c(Blist, M, T, thetalist0, eomg, ev):

    maxiterations = 200
    success = False
    thf =[]
    qf = []
    thf.append(thetalist0)

    toe = r.FKinBody(M, Blist, thetalist0[3:])
    tbe = r.matmult(Tbo,toe)

    tsb = [[cos(thetalist0[0]), -sin(thetalist0[0]), 0, thetalist0[1]],[sin(thetalist0[0]), cos(thetalist0[0]),  0, thetalist0[2]],[0, 0, 1, 0.0963],[0, 0, 0, 1]]

    Vb = r.MatrixLog6(r.matmult(r.TransInv(r.matmult(tsb,tbe)),T))
    print Vb

    wb  = r.Magnitude ([Vb[0],Vb[1],Vb[2]])
    print wb
    vb = r.Magnitude ([Vb[3],Vb[4],Vb[5]])
    print vb
    
    while (wb>eomg or vb>ev):

       	toe = r.FKinBody(M, Blist, thetalist0[3:])
       	tbe = r.matmult(Tbo,toe)

       	tsb = [[cos(thetalist0[0]), -sin(thetalist0[0]), 0, thetalist0[1]],[sin(thetalist0[0]), cos(thetalist0[0]),  0, thetalist0[2]],[0, 0, 1, 0.0963],[0, 0, 0, 1]]
       	Vb = r.MatrixLog6(r.matmult(r.TransInv(r.matmult(tsb,tbe)),T))
        	
    	wb  = r.Magnitude ([Vb[0],Vb[1],Vb[2]])
    	print wb
    	
    	vb = r.Magnitude ([Vb[3],Vb[4],Vb[5]])
    	print vb

    	jarm = r.JacobianBody(Blist, thetalist0[3:])
    	jbase = r.matmult(r.Adjoint(r.matmult(r.TransInv(toe),r.TransInv(Tbo))),F)
    	je = np.concatenate((jbase, jarm),axis=1)

    	uthed = r.matmult(np.linalg.pinv(je),Vb)
    	u = uthed[:4]

    	vb6 = r.matmult(F,u)[2:5]

    	thetader = np.hstack((vb6,uthed[4:]))

    	thetalist0 = np.add(thetalist0,thetader)
    	thf.append(thetalist0)

    	success = True

    vbold = thf[len(thf)-1]
    vbnew = [ [ vbold[1], vbold[2], vbold[0], vbold[3], vbold[4], vbold[5], vbold[6], vbold[7] ] ]

    with open("outputPartC.csv", "wb") as f:
		writer = csv.writer(f)
		writer.writerows(vbnew)

    return (np.round(100*thf[len(thf)-1]),success)

print hw5_c(Blist, Moe, X, thetalist0, eomg, ev)