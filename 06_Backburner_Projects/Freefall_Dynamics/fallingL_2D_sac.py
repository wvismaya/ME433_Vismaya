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

import sys

## Arguments for system damping
dampz = 0.
dampy = 0.
dampx = 0.

try:
    dampz = float(sys.argv[1])
except IndexError:
    dampz = 0.0  # no 4th index

try:
    dampx = float(sys.argv[3])
except IndexError:
    dampx = 0.0  # no 4th index

# Arguments for varing initial conditions
th_l1 = 0.  # Initial angle of the lift.
th_l2 = -0.5*mpi

try:
    th_l1 = float(sys.argv[4])
except IndexError:
    th_l1 = 0.05*mpi # no 4th index

try:
    th_l2 = float(sys.argv[5])
except IndexError:
    th_l2 = mpi - th_l1  # no 4th index

# Define the simulation parameters
segments = 1      # Number of segments in the scissor lift.
m_link = 1.0      # Mass of each link in the lift.
I_link = 1.0      # Rotational inertia of each link in the lift.
L_link = 5.0      # Length of each link in the lift.
m_slider = 1.0    # Mass of the top slider of the lift.
theta_0 = 0.0*mpi # Initial angle of the lift.

# define time parameters:
control_frequency = 200.  #Hz
dt = 1./control_frequency #0.0167
tf = 5.0

def falling_links():
    """ Create a scissor lift according to the global simulation parameters. """
    def add_level(left, right, link=0):
        """
        Recursive function to build the scissor lift by attaching
        another set of links.  'left' and 'right' are the coordinate
        frames to attach the left and right links to.
        """
        if link == segments:  # Terminate the recusions
            return (left, right)

        # Create the base of the left link
        left = Frame(left, trep.RY, 'Theta_Right')
        # "L%02d" % link
        left.config.q = th_l1
        # Create a frame at the middle of the link to attach the link's mass.
        left_mid = Frame(left, trep.TX, L_link/2.0)
        left_mid.set_mass(m_link, I_link, I_link, I_link)
        # Add the end of the link.
        left_end = Frame(left, trep.TX, L_link)

        right = Frame(right, trep.RY, 'Theta_pax')                              
        right.config.q = th_l2
        right_mid = Frame(right, trep.TX, L_link/2.0)
        right_mid.set_mass(m_link, I_link, I_link, I_link)
        right_end = Frame(right, trep.TX, L_link)

        # Join the two links at the middle.
        #trep.constraints.PointToPoint2D(system, 'xz', left_mid, right_mid)

        # Add a new level.  Note that left and right switch each time
        return add_level(right_end, left_end, link+1)

    # Create the new system
    system = trep.System()
    trep.potentials.Gravity(system, name="Gravity")
    # Add free fall variable
    drop = Frame(system.world_frame, trep.TZ, "DROP")
    drop.config.q = 0. #L_link*math.cos(theta_0)
    drop.set_mass(0.0)

    # Add the top slider
    slide = Frame(drop, trep.TX, "SLIDE")
    slide.config.q = 0. #L_link*math.cos(theta_0)
    slide.set_mass(0.0)
    # Create all the links in the system.
    add_level(slide, slide)
    # The scissor lift should satisfy the constraints, but call
    # satisfy_constraints() in case it needs to be nudged a little
    # from numerical error.
    trep.forces.Damping(system, 0.0, {'DROP' : dampz, 'SLIDE' : dampx})
    # trep.forces.ConfigForce(system, 'Theta_Right', 'rtorque') # Add input
    trep.forces.ConfigForce(system, 'DROP', 'dtorque') # Add input
    system.satisfy_constraints()
    return system

# Create the scissor lift
system = falling_links()

def proj_func(x):
    a = system.get_config('Theta_Right').index
    b = system.get_config('Theta_pax').index

    x[a] = np.fmod(x[a]+np.pi, 2.0*np.pi)
    if(x[a] < -0.001):
        x[a] = x[a]+2.0*np.pi
    x[a] = x[a]-np.pi

    x[b] = np.fmod(x[b]+np.pi, 2.0*np.pi)
    if(x[b] < -0.001):
        x[b] = x[b]+2.0*np.pi
    x[b] = x[b]-np.pi

def des_func(t, x, xdes):
    a = system.get_config('Theta_Right').index
    xdes[a] = -mpi*0.5 # Should be pi/2 wrt above value or to be held at -pi/2 constantly
    #x[1] = np.fmod(x[1]+np.pi, 2.0*np.pi)
    #xdes[3] = 0.  # Can be any value

# Instantiate the system
sacsys = sactrep.Sac(system)

sacsys.T = 1.2
sacsys.lam = -5
sacsys.maxdt = 0.2
sacsys.ts = dt
# There is only 1 input 'tl-torque'
sacsys.usat = [[1000, -1000]]
sacsys.calc_tm = 0.0
sacsys.u2search = True
sacsys.Q = np.diag([0., 0., 2., 20., 0., 0., 0., 0.]) # z, x, th1, th2, ,zd, xd, th1d, th2d
sacsys.P = np.diag([0,0, 0,0, 0,0, 0,0])
sacsys.R = 0.3*np.identity(1)

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
q = np.hstack((system.q[0], system.q[1],  system.q[2], system.q[3] ))
u = np.hstack((system.u ))
while sacsys.time < tf:
    sacsys.step()
    q = np.vstack(( q, np.hstack((system.q[0], system.q[1],  system.q[2], system.q[3] ))))
    u = np.vstack((u, np.hstack((system.u) )))
    if np.abs(sacsys.time%1)<dt:
        print "time = ",sacsys.time

print(system.q[3])
nn = np.shape(np.array(u))[0]
print(nn)
# pylab.plot(np.linspace(0,tf, num=nn), zip(*q)[system.get_config('Theta_Left').index])
# pylab.plot(np.linspace(0,tf, num=nn), zip(*q)[system.get_config('Theta_Right').index])
pylab.plot(np.linspace(0,tf, num=nn), zip(*q)[system.get_config('Theta_pax').index])
pylab.plot(np.linspace(0,tf, num=nn), np.pi*np.ones(nn))
# pylab.plot(np.linspace(0,tf, num=nn), 0.*np.pi*np.ones(nn))
# #pylab.plot(np.linspace(0,tf, num=nn), u)
pylab.legend(["Theta_Left","Theta_Right", "PI"])
pylab.show()
# sacsys.get_final_traj()
# sacsys.save()
# # Visualize the system in action
# visual.visualize_3d([ visual.VisualItem3D(system, np.linspace(0,tf, num=nn) , q) ])

# Create a visualization of the scissor lift.
visual.visualize_2d([visual.VisualItem2D(system, np.linspace(0,tf, num=nn), q, plane='XZ')])