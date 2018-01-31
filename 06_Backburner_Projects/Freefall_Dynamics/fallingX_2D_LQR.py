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
t0 = 0.0 # Initial time
zdes = 10.
qBar = [zdes, 0., 0., 0.] # Desired configuration
# z x t1 t2 zd xd t1d t2d
Q = np.diag([200., 20., 200.,  200., 0., 0., 0., 0.])
x0 = 0.*mpi + 0.01
y0 = mpi
#np.eye(14) # Cost weights for states
# fx fz my
R = np.diag([3., 3.])#0.3*np.eye(4) # Cost weights for inputs

# Define the simulation parameters
segments = 1.0      # Number of segments in the scissor lift.
m_link = 160.0 # Total should be 240.      # Mass of each link in the lift.
I_link = m_link/10.      # Rotational inertia of each link in the lift.
L_link = 3.54      # Length of each link in the lift.
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
    fallframe.config.q = 0. #L_link*math.cos(theta_0)
    #fallframe = Frame(fallframe, trep.TY, "WIGGLE")
    fallframe = Frame(fallframe, trep.TX, "SLIDE")
    fallframe.set_mass(0.)
    # Create all the links in the system.
    #add_level(fallframe, fallframe)
    # Create the base of the left link
    #left =
    left_mid = Frame(fallframe, trep.RY, 'PITCH')
    #left = Frame(left, trep.RX, 'ROLL')
    #left_mid = Frame(left, trep.RZ, 'YAW')
    left_mid.set_mass(m_link, I_link, I_link, I_link)
    lcf = 1./8.
    # "L%02d" % link
    # Create a frame at the middle of the link to attach the link's mass.
    left_leg1 = Frame(left_mid, trep.RZ, mpi/4.)
    left_leg1 = Frame(left_mid, trep.RZ, mpi/4.)
    left_leg1 = Frame(left_leg1, trep.TX, -L_link/2.0)
    left_leg1.set_mass(lcf*m_link, lcf*I_link, lcf*I_link, lcf*I_link)
    # Add the end of the link.
    left_leg2 = Frame(left_mid, trep.RZ, mpi/2 + mpi/4.)
    left_leg2 = Frame(left_leg2, trep.TX, L_link/2.0)
    left_leg2.set_mass(lcf*m_link, lcf*I_link, lcf*I_link, lcf*I_link)
    # Create a frame at the middle of the link to attach the link's mass.
    left_leg3 = Frame(left_mid, trep.RZ, mpi + mpi/4.)
    left_leg3 = Frame(left_leg3, trep.TX, -L_link/2.0)
    left_leg3.set_mass(lcf*m_link, lcf*I_link, lcf*I_link, lcf*I_link)
    # Add the end of the link.
    left_leg4 = Frame(left_mid, trep.RZ, 3*mpi/2 + mpi/4.)
    left_leg4 = Frame(left_leg4, trep.TX, L_link/2.0)
    left_leg4.set_mass(lcf*m_link, lcf*I_link, lcf*I_link, lcf*I_link)
    ###################################################################
    # # Passenger frame
    #right = Frame(left_mid, trep.RX, 'PAXx')
    #right.config.q = -0.5*mpi #left.config.q - mpi*0.5
    right = Frame(left_mid, trep.RY, 'PAXy')
    pcf = 100./m_link
    plcf = 2.074/L_link
    stomachat = 8.0
    #right.config.q = -0.5*mpi #left.config.q - mpi*0.5
    right_mid = Frame(right, trep.TZ, plcf*L_link/stomachat)
    right_mid.set_mass(pcf*m_link, 0.1, plcf*pcf*I_link, 0.1)
    right_end = Frame(right_mid, trep.TZ, (stomachat-1)*plcf*L_link/stomachat)
    # Join the two links at the middle.
    # trep.constraints.PointToPoint2D(system, 'xz', left_mid, right_mid)
    # 6.15
    trep.forces.Damping(system, 0.0, {'DROP' : 0, 'SLIDE' : 5.2, 'PITCH' : 0. , 'PAXy' : 100.})
    trep.forces.BodyWrench(system, fallframe, wrench=(0., 0., 'forcez', 'moment_pitch', 0))
    # the passengers response motion
    # trep.forces.ConfigForce(system, 'PAX', 'response-torque') # Add input
    system.satisfy_constraints()
    
    return system

system = falling_links()

# Create and initialize the variational integrator
mvi = trep.MidpointVI(system)
mvi.initialize_from_configs(t0, np.array([0.,0., 0., x0]), t0+dt, np.array([0.,0., 0., x0]))

# Create discrete system
TVec = np.arange(t0, tf+dt, dt) # Initialize discrete time vector
dsys = trep.discopt.DSystem(mvi, TVec) # Initialize discrete system
xBar = dsys.build_state(qBar) # Create desired state configuration

# Design linear feedback controller
Qd = np.zeros((len(TVec), dsys.system.nQ)) # Initialize desired configuration trajectory
dropIndex = dsys.system.get_config('DROP').index
#thetaIndexx = dsys.system.get_config('PAXx').index # Find index of theta config variable
thetaIndexy = dsys.system.get_config('PAXy').index # Find index of theta config variable
#rollIndex = dsys.system.get_config('ROLL').index # Find index of theta config variable
pitchIndex = dsys.system.get_config('PITCH').index # Find index of theta config variable
for i,t in enumerate(TVec):
    #Qd[i, thetaIndexx] = 0. # Set desired configuration trajectory
    Qd[i, thetaIndexy] = 0. # Set desired configuration trajectory
    #Qd[i, rollIndex] = 0. # Set desired configuration trajectory
    Qd[i, pitchIndex] = 0. # Set desired configuration trajectory
    #Qd[i, dropIndex] = 2. # Set desired configuration trajectory
    (Xd, Ud) = dsys.build_trajectory(Qd) # Set desired state and input trajectory

Qk = lambda k: Q # Create lambda function for state cost weights
Rk = lambda k: R # Create lambda function for input cost weights
KVec = dsys.calc_feedback_controller(Xd, Ud, Qk, Rk) # Solve for linear feedback controller gain
KStabilize = dot(np.array([100., 100.]), KVec[0]) # Use only use first value to approximate infinite-horizon optimal controller gain
# dot(np.array([10., 10.]),  )
# Reset discrete system state
dsys.set(np.array([0., 0., 0., x0, 0.,0.,0., 0.]), np.array([0.,0]), 0)

# Helper functions
def wrapTo2Pi(ang):
    return ang % (2*pi)

def wrapToPi(ang):
    return (ang + pi) % (2*pi) - pi

def wrapToZ(ang):
    return (ang  % (zdes))

# Simulate the system forward
T = [mvi.t1] # List to hold time values
Q = [mvi.q1] # List to hold configuration values
X = [dsys.xk] # List to hold state values
U = [] # List to hold input values
uhigh = m_link*9.81*2.
ulow = -uhigh
while mvi.t1 < tf-dt:
    x = dsys.xk # Grab current state
    xTilde = np.array([ x[0]-xBar[0], x[1]-xBar[1], wrapToPi(x[2] - xBar[2]), wrapToPi(x[3] - xBar[3]), x[4], x[5], x[6], x[7] ])
    #x - xBar # Compare to desired state
    u = -dot(KStabilize, xTilde) # Calculate input
    u =  np.minimum(np.array([uhigh, uhigh*0.5]), np.maximum(np.array([ulow, ulow*0.5]), u ) )
    dsys.step(u) # Step the system forward by one time step
    T.append(mvi.t1) # Update lists
    Q.append(mvi.q1)
    X.append(x)
    U.append(u)

# Plot results
ax1 = pylab.subplot(311)
#pylab.plot(T, zip(*X)[0])
pylab.plot(T, zip(*X)[2])
pylab.plot(T, zip(*X)[3])
#pylab.plot(T, zip(*X)[1])
#pylab.plot(T, zip(*X)[0])
pylab.title("Linear Feedback Controller")
pylab.ylabel("X")
pylab.legend(["pitch","pax"])
pylab.subplot(312, sharex=ax1)
pylab.plot(T[1:], U)
pylab.xlabel("T")
pylab.ylabel("U")
pylab.legend(["u"])
pylab.subplot(313, sharex=ax1)
pylab.plot(T, zip(*X)[1])
pylab.plot(T, zip(*X)[0])
pylab.xlabel("T")
pylab.ylabel("Z")
pylab.legend(["X", "Z"])
pylab.show()

visual.visualize_2d([visual.VisualItem2D(system, T, X, plane='XZ')])
# visual.visualize_2d([visual.VisualItem2D(system, np.linspace(0,tf, num=nn), q, plane='YZ')])
# visual.visualize_2d([visual.VisualItem2D(system, np.linspace(0,tf, num=nn), q, plane='XY')])