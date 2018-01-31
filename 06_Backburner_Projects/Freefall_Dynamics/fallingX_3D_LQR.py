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
qBar = [zdes, 0., 0.,   0., 0., 0.,   0.] # Desired configuration
# z y x tr tp ty tpx zd xd t1d t2d
Q = np.diag([200., 20., 20., 200., 200., 200., 200., 0., 0., 0., 0., 0., 0., 0.])
x0 = 0.*mpi + 0.01
y0 = mpi
#np.eye(14) # Cost weights for states
# fx fz my
R = np.diag([0.5, 0.3, 0.3, 0.3, 0.4])#0.3*np.eye(4) # Cost weights for inputs

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
tf = 50.0

def falling_links():
    # Create the new system
    system = trep.System()
    trep.potentials.Gravity(system, name="Gravity")
    # Add free fall variable
    fallframe = Frame(system.world_frame, trep.TZ, "DROP")
    fallframe.config.q = 0. #L_link*math.cos(theta_0)
    fallframe = Frame(fallframe, trep.TY, "WIGGLE")
    fallframe = Frame(fallframe, trep.TX, "SLIDE")
    fallframe.set_mass(0.)
    # Create all the links in the system.
    #add_level(fallframe, fallframe)
    # Create the base of the left link
    left = Frame(fallframe, trep.RX, 'ROLL')
    left = Frame(left, trep.RY, 'PITCH')
    left_mid = Frame(left, trep.RZ, 'YAW')
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
    right_mid.set_mass(pcf*m_link, plcf*pcf*I_link, plcf*pcf*I_link, plcf*pcf*I_link)
    right_end = Frame(right_mid, trep.TZ, (stomachat-1)*plcf*L_link/stomachat)
    # Join the two links at the middle.
    # trep.constraints.PointToPoint2D(system, 'xz', left_mid, right_mid)
    # 6.15
    trep.forces.Damping(system, 0.0, {'DROP' : 2., 'WIGGLE' : 1., 'SLIDE' : 1., 'ROLL' : 1., 'PITCH' : 1., 'YAW' : 1. , 'PAXy' : 100.})
    #trep.forces.BodyWrench(system, fallframe, wrench=(0., 0., 'forcez', 'm1', 'm2', 'm3'))
    # the passengers response motion
    trep.forces.ConfigForce(system, 'DROP', 'fz') # Add input
    trep.forces.ConfigForce(system, 'PITCH', 'm1') # Add input
    trep.forces.ConfigForce(system, 'ROLL', 'm2') # Add input
    trep.forces.ConfigForce(system, 'YAW', 'm3') # Add input
    trep.forces.ConfigForce(system, 'PAXy', 'fpy') # Add input
    system.satisfy_constraints()
    
    return system

system = falling_links()

# Create and initialize the variational integrator
mvi = trep.MidpointVI(system)
mvi.initialize_from_configs(t0, np.array([0.,0., 0.,   0.,0.,0.,  x0]), t0+dt, np.array([0.,0., 0.,   0.,0.,0.,   x0]))

# Create discrete system
TVec = np.arange(t0, tf+dt, dt) # Initialize discrete time vector
dsys = trep.discopt.DSystem(mvi, TVec) # Initialize discrete system
xBar = dsys.build_state(qBar) # Create desired state configuration

# Design linear feedback controller
Qd = np.zeros((len(TVec), dsys.system.nQ)) # Initialize desired configuration trajectory
dropIndex = dsys.system.get_config('DROP').index
#thetaIndexx = dsys.system.get_config('PAXx').index # Find index of theta config variable
thetaIndexy = dsys.system.get_config('PAXy').index # Find index of theta config variable
rollIndex = dsys.system.get_config('ROLL').index # Find index of theta config variable
pitchIndex = dsys.system.get_config('PITCH').index # Find index of theta config variable
for i,t in enumerate(TVec):
    #Qd[i, thetaIndexx] = 0. # Set desired configuration trajectory
    Qd[i, thetaIndexy] = 5.*mpi/180. # Set desired configuration trajectory
    Qd[i, rollIndex] = 0. # Set desired configuration trajectory
    Qd[i, pitchIndex] = 0. # Set desired configuration trajectory
    Qd[i, dropIndex] = zdes # Set desired configuration trajectory
    (Xd, Ud) = dsys.build_trajectory(Qd) # Set desired state and input trajectory

Qk = lambda k: Q # Create lambda function for state cost weights
Rk = lambda k: R # Create lambda function for input cost weights
KVec = dsys.calc_feedback_controller(Xd, Ud, Qk, Rk) # Solve for linear feedback controller gain
KStabilize =  100. * KVec[0] # Use only use first value to approximate infinite-horizon optimal controller gain
#print[KStabilize]
# dot(np.array([10., 10.]),  )
# Reset discrete system state
dsys.set(np.array([0., 0., 0.,   0.,0.,0.,  x0,   0.,0.,0.,   0.,0.,0.,  0.]), np.array([0.,0., 0.,0., 0]), 0)

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
uhigh = m_link*9.81*3.
ulow = -uhigh
mcf = 0.3
mpcf = 0.08
while mvi.t1 < tf-dt:
    x = dsys.xk # Grab current state
    xTilde = np.array([ x[0]-xBar[0], x[1]-xBar[1], x[2]-xBar[2], wrapToPi(x[3] - xBar[3]), wrapToPi(x[4] - xBar[4]), wrapToPi(x[5] - xBar[5]), wrapToPi(x[6] - xBar[6]), x[7], x[8], x[9], x[10], x[11] , x[12]  , x[13] ])
    #x - xBar # Compare to desired state
    #print(KStabilize)
    u = -dot(KStabilize, xTilde) # Calculate input
    #print(u)
    u =  np.minimum(np.array([uhigh, uhigh*mcf, uhigh*mcf, uhigh*mcf, uhigh*mpcf]), np.maximum(np.array([ulow, ulow*mcf, ulow*mcf, ulow*mcf, ulow*mpcf]), u ) )
    
    dsys.step(u) # Step the system forward by one time step
    T.append(mvi.t1) # Update lists
    Q.append(mvi.q1)
    X.append(x)
    U.append(u)

# Plot results
ax1 = pylab.subplot(311)
#pylab.plot(T, zip(*X)[0])
pylab.plot(T, zip(*X)[3])
pylab.plot(T, zip(*X)[4])
pylab.plot(T, zip(*X)[6])
#pylab.plot(T, zip(*X)[1])
#pylab.plot(T, zip(*X)[0])
pylab.title("Linear Feedback Controller")
pylab.ylabel("X")
pylab.legend(["roll", "pitch","pax"])
pylab.subplot(312, sharex=ax1)
pylab.plot(T[1:], zip(*U)[0])
pylab.plot(T[1:], zip(*U)[1])
pylab.plot(T[1:], zip(*U)[2])
pylab.plot(T[1:], zip(*U)[3])
#pylab.plot(T[1:], zip(*U)[4])
pylab.xlabel("T")
pylab.ylabel("U")
pylab.legend(['fz', 'm1','m2','m3','pax-torque'])
pylab.subplot(313, sharex=ax1)
pylab.plot(T, zip(*X)[2])
pylab.plot(T, zip(*X)[1])
pylab.plot(T, zip(*X)[0])
pylab.xlabel("T")
pylab.ylabel("Z")
pylab.legend(["X", "Y", "Z"])
pylab.show()

visual.visualize_2d([visual.VisualItem2D(system, T, X, plane='XZ')])
# visual.visualize_2d([visual.VisualItem2D(system, np.linspace(0,tf, num=nn), q, plane='YZ')])
# visual.visualize_2d([visual.VisualItem2D(system, np.linspace(0,tf, num=nn), q, plane='XY')])