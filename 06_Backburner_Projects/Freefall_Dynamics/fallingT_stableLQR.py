# linearFeedbackController.py

# Import necessary python modules
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
import math
from math import pi
import numpy as np
from numpy import dot
import trep
import trep.discopt
from trep import tx, ty, tz, rx, ry, rz
import pylab

# Build a pendulum system
m = 1.0 # Mass of pendulum
l = 1.0 # Length of pendulum
q0 = 3./4.*pi # Initial configuration of pendulum
t0 = 0.0 # Initial time
tf = 5.0 # Final time
dt = 0.1 # Sampling time
qBar = pi # Desired configuration
Q = np.diag([1., 1., 2., 200.,  200., 1., 80., 80., 0., 0., 0., 0., 0., 0., 0., 0.])
x0 = mpi
y0 = mpi
#np.eye(14) # Cost weights for states
R = np.diag([0.3, 0.3, 0.3, 0.3])#0.3*np.eye(4) # Cost weights for inputs

# Define the simulation parameters
segments = 1.0      # Number of segments in the scissor lift.
m_link = 1.0      # Mass of each link in the lift.
I_link = 1.0      # Rotational inertia of each link in the lift.
L_link = 5.0      # Length of each link in the lift.
m_slider = 1.0    # Mass of the top slider of the lift.
theta_0 = 0.0*mpi # Initial angle of the lift.

# Define time parameters:
control_frequency = 100. #Hz
dt = 1./control_frequency
tf = 10.0

def falling_links():
    # Create the new system
    system = trep.System()
    trep.potentials.Gravity(system, name="Gravity")
    # Add free fall variable
    fallframe = Frame(system.world_frame, trep.TZ, "DROP")
    fallframe.config.q = 10. #L_link*math.cos(theta_0)
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
    right = Frame(left_mid, trep.RX, 'PAXx')
    #right.config.q = -0.5*mpi #left.config.q - mpi*0.5
    right = Frame(right, trep.RY, 'PAXy')
    #right.config.q = -0.5*mpi #left.config.q - mpi*0.5
    right_mid = Frame(right, trep.TX, L_link/9.0)
    right_mid.set_mass(0.33*m_link, 0.33*I_link, 0.33*I_link, 0.33*I_link)
    right_end = Frame(right, trep.TX, L_link/3.0)
    # Join the two links at the middle.
    # trep.constraints.PointToPoint2D(system, 'xz', left_mid, right_mid)
    trep.forces.Damping(system, 0.0, {'DROP' : 0., 'WIGGLE' : 0., 'SLIDE' : 0., 'PAXx' : 0., 'PAXy' : 0.})
    trep.forces.BodyWrench(system, fallframe, wrench=(0., 0., 'forcez', 'moment1', 'moment2', 'moment3'))
    # the passengers response motion
    # trep.forces.ConfigForce(system, 'PAX', 'response-torque') # Add input
    system.satisfy_constraints()
    
    return system

system = falling_links()

# Create and initialize the variational integrator
mvi = trep.MidpointVI(system)
mvi.initialize_from_configs(t0, np.array([0., 0.,0., 0.,0., 0., x0, y0]), t0+dt, np.array([0., 0.,0., 0.,0., 0., x0, y0]))

# Create discrete system
TVec = np.arange(t0, tf+dt, dt) # Initialize discrete time vector
dsys = trep.discopt.DSystem(mvi, TVec) # Initialize discrete system
xBar = dsys.build_state(qBar) # Create desired state configuration

# Design linear feedback controller
Qd = np.zeros((len(TVec), dsys.system.nQ)) # Initialize desired configuration trajectory
dropIndex = dsys.system.get_config('DROP').index
thetaIndexx = dsys.system.get_config('PAXx').index # Find index of theta config variable
thetaIndexy = dsys.system.get_config('PAXy').index # Find index of theta config variable
rollIndex = dsys.system.get_config('ROLL').index # Find index of theta config variable
pitchIndex = dsys.system.get_config('PITCH').index # Find index of theta config variable
for i,t in enumerate(TVec):
    Qd[i, thetaIndexx] = 0. # Set desired configuration trajectory
    Qd[i, thetaIndexy] = 0 # Set desired configuration trajectory
    #Qd[i, rollIndex] = 0. # Set desired configuration trajectory
    #Qd[i, pitchIndex] = 0. # Set desired configuration trajectory
    #Qd[i, dropIndex] = 2. # Set desired configuration trajectory
    (Xd, Ud) = dsys.build_trajectory(Qd) # Set desired state and input trajectory

Qk = lambda k: Q # Create lambda function for state cost weights
Rk = lambda k: R # Create lambda function for input cost weights
KVec = dsys.calc_feedback_controller(Xd, Ud, Qk, Rk) # Solve for linear feedback controller gain
KStabilize = 3.*KVec[0] # Use only use first value to approximate infinite-horizon optimal controller gain

# Reset discrete system state
dsys.set(np.array([0., 0., 0., 0.,0., 0., x0, y0, 0.,0.,0., 0.,0., 0.,0., 0.]), np.array([0.,0.,0.,0.]), 0)

# Simulate the system forward
T = [mvi.t1] # List to hold time values
Q = [mvi.q1] # List to hold configuration values
X = [dsys.xk] # List to hold state values
U = [] # List to hold input values
while mvi.t1 < tf-dt:
    x = dsys.xk # Grab current state
    xTilde = x - xBar # Compare to desired state
    u = -dot(KStabilize, xTilde) # Calculate input
    dsys.step(u) # Step the system forward by one time step
    T.append(mvi.t1) # Update lists
    Q.append(mvi.q1)
    X.append(x)
    U.append(u)

# Plot results
ax1 = pylab.subplot(211)
#pylab.plot(T, zip(*X)[0])
pylab.plot(T, zip(*X)[6])
pylab.plot(T, zip(*X)[7])
pylab.plot(T, zip(*X)[3])
pylab.plot(T, zip(*X)[4])
pylab.title("Linear Feedback Controller")
pylab.ylabel("X")
pylab.legend(["qd","p"])
pylab.subplot(212, sharex=ax1)
pylab.plot(T[1:], U)
pylab.xlabel("T")
pylab.ylabel("U")
pylab.legend(["u"])
pylab.show()

visual.visualize_2d([visual.VisualItem2D(system, T, X, plane='XZ')])
# visual.visualize_2d([visual.VisualItem2D(system, np.linspace(0,tf, num=nn), q, plane='YZ')])
# visual.visualize_2d([visual.VisualItem2D(system, np.linspace(0,tf, num=nn), q, plane='XY')])