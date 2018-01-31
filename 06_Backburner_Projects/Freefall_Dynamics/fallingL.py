#!/usr/bin/python

# Simulation of a scissor lift.  The number of links in the lift can
# be changed.

import sys
import trep
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
    dampy = float(sys.argv[2])
except IndexError:
    dampy = 0.0  # no 4th index

try:
    dampx = float(sys.argv[3])
except IndexError:
    dampx = 0.0  # no 4th index

# Arguments for varing initial conditions
th_l1 = 0.05*mpi  # Initial angle of the lift.
th_l2 = mpi - th_l1

try:
    th_l1 = float(sys.argv[4])
except IndexError:
    th_l1 = (-0.5*mpi+0.25*mpi) # no 4th index

try:
    th_l2 = float(sys.argv[5])
except IndexError:
    th_l2 = th_l1 + 0.5*mpi  # no 4th index

# Define the simulation parameters
segments = 1      # Number of segments in the scissor lift.
m_link = 1.0      # Mass of each link in the lift.
I_link = 1.0      # Rotational inertia of each link in the lift.
L_link = 5.0      # Length of each link in the lift.
m_slider = 1.0    # Mass of the top slider of the lift.
tf = 10.0         # Duration of the simulation
dt = 0.01         # Time step of the simulation

def simulate_system(system):
    """
    Simulates the system from its current configuration with no
    initial velocity for the duration and time step specified by global variables.
    """
    # Extract the current configuration into a tuple to use as the two
    # initial configurations of the variational integrator.
    q0 = system.q

    # Create and initialize the variational integrator.
    mvi = trep.MidpointVI(system)
    mvi.initialize_from_configs(0.0, q0, dt, q0)

    # Run the simulation and save the results into the lists 't' and 'q'.
    q = [mvi.q1]
    t = [mvi.t1]
    while mvi.t1 < tf:
        mvi.step(mvi.t2+dt)
        q.append(mvi.q1)
        t.append(mvi.t1) 
        # Print out progress during the simulation.
        if abs(mvi.t1 - round(mvi.t1)) < dt/2.0:
            print "t =",mvi.t1

    return (t, q)

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
        left = Frame(left, trep.RY, 'Theta_Left')
        # "L%02d" % link
        left.config.q = th_l1
        # Create a frame at the middle of the link to attach the link's mass.
        left_mid = Frame(left, trep.TX, L_link/2.0)
        left_mid.set_mass(m_link, I_link, I_link, I_link)
        # Add the end of the link.
        left_end = Frame(left, trep.TX, L_link)

        right = Frame(right, trep.RY, 'Theta_Right')                              
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
    drop.config.q = 10. #L_link*math.cos(theta_0)
    drop.set_mass(0.0)

    wiggle = Frame(drop, trep.TY, "WIGGLE")
    wiggle.config.q = 10. #L_link*math.cos(theta_0)
    wiggle.set_mass(0.0)
    # Add the top slider
    slide = Frame(wiggle, trep.TX, "SLIDE")
    slide.config.q = 0. #L_link*math.cos(theta_0)
    slide.set_mass(0.0)
    # Create all the links in the system.
    add_level(slide, slide)
    # The scissor lift should satisfy the constraints, but call
    # satisfy_constraints() in case it needs to be nudged a little
    # from numerical error.
    trep.forces.Damping(system, 0.0, {'DROP' : dampz, 'WIGGLE' : dampy, 'SLIDE' : dampx})
    system.satisfy_constraints()
    return system

# Create the scissor lift
system = falling_links()
# Simulate the system
(t, q) = simulate_system(system)

# Create a visualization of the scissor lift.
visual.visualize_2d([visual.VisualItem2D(system, t, q, plane='XZ')])