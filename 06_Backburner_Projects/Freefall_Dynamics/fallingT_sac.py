import numpy as np
import trep
import sactrep
import pylab
import sys
from trep import Frame
import trep.visual as visual
import math
from math import pi as mpi
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *

# Define the simulation parameters
segments = 1      # Number of segments in the scissor lift.
m_link = 1.0      # Mass of each link in the lift.
I_link = 1.0      # Rotational inertia of each link in the lift.
L_link = 5.0      # Length of each link in the lift.
m_slider = 1.0    # Mass of the top slider of the lift.
theta_0 = 0.0*mpi # Initial angle of the lift.

# Define time parameters:
control_frequency = 200. #Hz
dt = 0.0167 #1./control_frequency
tf = 3.0

def falling_links():
    # Create the new system
    system = trep.System()
    trep.potentials.Gravity(system, name="Gravity")
    # Add free fall variable
    fallframe = Frame(system.world_frame, trep.TZ, "DROP")
    #fallframe.config.q = 10. #L_link*math.cos(theta_0)
    fallframe = Frame(fallframe, trep.TY, "WIGGLE")
    fallframe = Frame(fallframe, trep.TX, "SLIDE")
    fallframe.set_mass(0.0)
    # Create all the links in the system.
    #add_level(fallframe, fallframe)
    # Create the base of the left link
    left = Frame(fallframe, trep.RX, 'ROLL')
    left = Frame(left, trep.RY, 'PITCH')
    left_mid = Frame(left, trep.RZ, 'YAW')
    left_mid.set_mass(m_link, I_link, I_link, I_link)
    # "L%02d" % link
    # Create a frame at the middle of the link to attach the link's mass.
    left_leg1 = Frame(left_mid, trep.RZ, mpi/4.)
    left_leg1 = Frame(left_leg1, trep.TX, -L_link/2.0)
    left_leg1.set_mass(0.25*m_link, 0.25*I_link, 0.25*I_link, 0.25*I_link)
    # Add the end of the link.
    left_leg2 = Frame(left_mid, trep.RZ, mpi/2 + mpi/4.)
    left_leg2 = Frame(left_leg2, trep.TX, L_link/2.0)
    left_leg2.set_mass(0.25*m_link, 0.25*I_link, 0.25*I_link, 0.25*I_link)
    # Create a frame at the middle of the link to attach the link's mass.
    left_leg3 = Frame(left_mid, trep.RZ, mpi + mpi/4.)
    left_leg3 = Frame(left_leg3, trep.TX, -L_link/2.0)
    left_leg3.set_mass(0.25*m_link, 0.25*I_link, 0.25*I_link, 0.25*I_link)
    # Add the end of the link.
    left_leg4 = Frame(left_mid, trep.RZ, 3*mpi/2 + mpi/4.)
    left_leg4 = Frame(left_leg4, trep.TX, L_link/2.0)
    left_leg4.set_mass(0.25*m_link, 0.25*I_link, 0.25*I_link, 0.25*I_link)
    ###################################################################
    # # Passenger frame
    # right = Frame(left_mid, trep.RY, 'PAX')
    # right.config.q = -0.5*mpi + 0.05 #left.config.q - mpi*0.5
    # right_mid = Frame(right, trep.TX, L_link/9.0)
    # right_mid.set_mass(0.5*m_link, 0.5*I_link, 0.5*I_link, 0.5*I_link)
    # right_end = Frame(right, trep.TX, L_link/3.0)
    # Join the two links at the middle.
    # trep.constraints.PointToPoint2D(system, 'xz', left_mid, right_mid)
    trep.forces.Damping(system, 0.0, {'DROP' : 0.05, 'WIGGLE' : 0.18, 'SLIDE' : 0.18})
    trep.forces.BodyWrench(system, fallframe, wrench=(0.1, 0.1, 'forcez', 'moment1', 'moment2', 'moment3'))
    # the passengers response motion
    # trep.forces.ConfigForce(system, 'PAX', 'response-torque') # Add input
    system.satisfy_constraints()
    
    return system

system = falling_links()

def proj_func(x):
    if(x[3] < -0.001):
        x[3] = x[3]+2.0*np.pi
    x[3] = np.fmod(x[3], np.pi)
    x[3] = x[3]

    if(x[4] < -0.001):
        x[4] = x[4]+2.0*np.pi
    x[4] = np.fmod(x[4], np.pi)
    x[4] = x[4]

def des_func(t, x, xdes):
    #x[1] = np.fmod(x[1]+np.pi, 2.0*np.pi)
    xdes[system.get_config('DROP').index] = 7.
    xdes[system.get_config('WIGGLE').index] = 0.
    xdes[system.get_config('SLIDE').index] = 0.
    # xdes[system.get_config('ROLL').index] = 0.  # Can be any value
    # xdes[system.get_config('PITCH').index] = 0.  # Can be any value
    # xdes[system.get_config('PAX').index] = -mpi*0.5 # Should be pi/2 wrt above value or to be held at -pi/2 constantly

# Instantiate the system
sacsys = sactrep.Sac(system)

sacsys.T = 1.2
sacsys.lam = -5
sacsys.maxdt = 0.2
sacsys.ts = dt
# There is only 1 input 'tl-torque'
sacsys.usat = [[100., -100.], [100., -100.], [100., -100.], [100., -100.]]
sacsys.calc_tm = 0.0
sacsys.u2search = True
sacsys.Q = np.diag([3., 3., 3., 1500., 1500, 1500., 0., 0., 0., 0., 0., 0.])
# z, y, x, roll, pitch, yaw, pax, zd, yd, xd, rolld, pitchd, yawd, paxd
sacsys.P = np.diag([0., 0.,0. ,0.,0.,0., 0.,0.,0. ,0.,0.,0.])
sacsys.R = np.diag([0.1, 0.1, 0.1, 0.1 ]) # liftForce, M1, M2, M3, passengerForce
# 
sacsys.set_proj_func(proj_func)
sacsys.set_xdes_func(des_func)

# set initial conditions:
q0 = system.q
#dq0 = system.dq
#system.q = q0
#system.dq = dq0

# init SAC:
sacsys.init()

# run loop:
q = np.hstack((system.q[0], system.q[1],  system.q[2], system.q[3], system.q[4], system.q[5],
               system.dq[0], system.dq[1], system.dq[2], system.dq[3], system.dq[4], system.dq[5]))
u = np.hstack((system.u[0], system.u[1], system.u[2], system.u[3]))
while sacsys.time < tf:
    sacsys.step()
    q = np.vstack((q, np.hstack((system.q[0], system.q[1], system.q[2], system.q[3], system.q[4], system.q[5],
                                 system.dq[0], system.dq[1], system.dq[2], system.dq[3], system.dq[4], system.dq[5]))))
    u = np.vstack((u, np.hstack((system.u[0], system.u[1], system.u[2], system.u[3]) )))
    if np.abs(sacsys.time%2)<dt:
        print "time = ",sacsys.time

nn = np.shape(np.array(u))[0]
print(nn)
pylab.plot(np.linspace(0,tf, num=nn), zip(*q)[system.get_config('DROP').index])
pylab.plot(np.linspace(0,tf, num=nn), zip(*q)[system.get_config('WIGGLE').index])
pylab.plot(np.linspace(0,tf, num=nn), zip(*q)[system.get_config('SLIDE').index])
# The angles
# pylab.plot(np.linspace(0,tf, num=nn), zip(*q)[system.get_config('ROLL').index])
# pylab.plot(np.linspace(0,tf, num=nn), zip(*q)[system.get_config('PITCH').index])
# pylab.plot(np.linspace(0,tf, num=nn), zip(*q)[system.get_config('YAW').index])
# pylab.plot(np.linspace(0,tf, num=nn), zip(*q)[system.get_config('PAX').index])
# pylab.plot(np.linspace(0,tf, num=nn), np.pi*np.ones(nn))
# pylab.plot(np.linspace(0,tf, num=nn), -np.pi*np.ones(nn))
# pylab.plot(np.linspace(0,tf, num=nn), 0.*np.pi*np.ones(nn))
# #pylab.plot(np.linspace(0,tf, num=nn), u)
pylab.legend(["z","y","x"])
pylab.show()
# sacsys.get_final_traj()
# sacsys.save()
# # Visualize the system in action
# visual.visualize_3d([ visual.VisualItem3D(system, np.linspace(0,tf, num=nn) , q) ])

# Create a visualization of the scissor lift.
# visual.visualize_2d([visual.VisualItem2D(system, np.linspace(0,tf, num=nn), q, plane='XZ')])
# visual.visualize_2d([visual.VisualItem2D(system, np.linspace(0,tf, num=nn), q, plane='YZ')])
# visual.visualize_2d([visual.VisualItem2D(system, np.linspace(0,tf, num=nn), q, plane='XY')])