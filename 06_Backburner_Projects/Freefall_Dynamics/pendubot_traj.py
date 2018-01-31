import numpy as np
import sys
import trep
from trep import tx, ty, tz, rx, ry, rz
import trep.discopt as discopt
import math
from math import sin, cos
from math import pi as mpi
import trep.visual as visual
import pylab

ad = mpi - (60*mpi/180)

## Time variables
t0 = 0
tf = 10
dt = 0.01

## The expected value of the pendulum to stabalise it upright which is at Pi ##

def build_system(torque_force=False):
    cart_mass = 1
    pendulum_length = 1.0
    pendulum_mass = 0.1

    system = trep.System()
    frames = [
    rz('theta', name="pendulumShoulder"), [
        tx(-pendulum_length, name="pendulumArm1", mass=pendulum_mass), [
            rz('x', name="pendulumElbow"), [
                tx(-pendulum_length, name="pendulumArm2", mass=pendulum_mass) ]]]]
    system.import_frames(frames)

    ## Because 2D system
    trep.potentials.Gravity(system, (0, -9.8, 0))
    trep.forces.Damping(system, 10)
    trep.forces.ConfigForce(system, 'x', 'x-force')
    if torque_force:
        trep.forces.ConfigForce(system, 'theta', 'theta-force')
    return system

def generate_desired_trajectory(system, t, amp= ad):
    qd = np.zeros((len(t), system.nQ))
    ## System nQ is 
    ## with get_config we get 
    theta_index = system.get_config('theta').index
    for i,t in enumerate(t):
        #if t >= 3.0 and t <= 7.0:
        qd[i, theta_index] = mpi
        #(1 - cos(2*mpi/4*(t-3.0)))*amp/2
    return qd

def generate_sys_energy(system, t, amp= ad):
    ed = np.zeros(len(t))
    ## System nQ is 
    ## with get_config we get 
    theta_index = system.get_config('theta').index
    for i,t in enumerate(t):
        ed[i] = system.total_energy()
    return ed

## Choose Q
def make_state_cost(dsys, base, x, theta):
    weight = base*np.ones((dsys.nX,))
    weight[system.get_config('x').index] = x
    weight[system.get_config('theta').index] = theta
    return np.diag(weight)

## Choose R
def make_input_cost(dsys, base, x, theta=None):
    weight = base*np.ones((dsys.nU,))
    if theta is not None:
        weight[system.get_input('theta-force').index] = theta
    weight[system.get_input('x-force').index] = 0
    return np.diag(weight)                    

# Build cart system with torque input on pendulum.
system = build_system(True)

# Create and initialize variational system
mvi = trep.MidpointVI(system)
# # I think if no explicit inital values 0 is considered.

# Create discrete system
T = [mvi.t1] # List to hold time values
t = np.arange(t0, tf, dt)
# System using variational Integrator and Time vector
dsys_a = discopt.DSystem(mvi, t)

# Generate an initial trajectory
(X,U) = dsys_a.build_trajectory()
for k in range(dsys_a.kf()):
    if k == 0:
        dsys_a.set(X[k], U[k], 0)
    else:
        dsys_a.step(U[k])
    X[k+1] = dsys_a.f()

############## OPTIMIZATION #############
# Generate cost function
qd = generate_desired_trajectory(system, t)
(Xd, Ud) = dsys_a.build_trajectory(qd)
Qcost = make_state_cost(dsys_a, 0.01, 0.01, 100.0)
Rcost = make_input_cost(dsys_a, 0.01, 0.01, 0.01)
cost = discopt.DCost(Xd, Ud, Qcost, Rcost)

optimizer = discopt.DOptimizer(dsys_a, cost)

# Perform the first optimization
optimizer.first_method_iterations = 10
finished, X, U = optimizer.optimize(X, U, max_steps=40)
print(abs(np.cumsum(U)[-1]/1000))

# Increase the cost of the torque input
cost.R = make_input_cost(dsys_a, 0.01, 0.01, 100.0)
optimizer.first_method_iterations = 10
finished, X, U = optimizer.optimize(X, U, max_steps=40)
print(abs(np.cumsum(U)[-1]/1000))

# Increase the cost of the torque input
cost.R = make_input_cost(dsys_a, 0.01, 0.01, 1000000.0)
optimizer.first_method_iterations = 10
finished, X, U = optimizer.optimize(X, U, max_steps=40)
print(abs(np.cumsum(U)[-1]/1000))

# The torque should be really tiny now, so we can hopefully use this trajectory as the initial trajectory of the real system.  

# Build a new system without the extra input
system = build_system(False)
mvi = trep.MidpointVI(system)
dsys_b = discopt.DSystem(mvi, t)

# Map the optimized trajectory for dsys_a to dsys_b
(X, U) = dsys_b.convert_trajectory(dsys_a, X, U)

# Simulate the new system starting from the initial condition of our last optimization and using the x-force input.
# kf is last avaible state the system can  be set to
for k in range(dsys_b.kf()):
    if k == 0:
        dsys_b.set(X[k], U[k], 0)
    else:
        dsys_b.step(U[k])
    X[k+1] = dsys_b.f()

# Generate a new cost function for the current system.
qd = generate_desired_trajectory(system, t)
(Xd, Ud) = dsys_b.build_trajectory(qd)
Qcost = make_state_cost(dsys_b, 0.01, 0.01, 100.0)
Rcost = make_input_cost(dsys_b, 0.01, 0.01)
cost = discopt.DCost(Xd, Ud, Qcost, Rcost)

optimizer = discopt.DOptimizer(dsys_b, cost)

# Perform the optimization on the real system
optimizer.first_method_iterations = 4
finished, X, U = optimizer.optimize(X, U, max_steps=40)

## We could print a converge plot here if we wanted to.
# dcost = np.array(optimizer.monitor.dcost_history.items()).T
# pylab.semilogy(dcost[0], -dcost[1])
# pylab.show()
ax1 = pylab.subplot(211)
pylab.plot(t, X[:,system.get_config('theta').index])
pylab.plot(t, ad*np.ones(len(t)))

pylab.title("Linear Feedback Controller")
pylab.ylabel("X")
#pylab.legend(["qd","p"])
pylab.grid()
pylab.subplot(212, sharex=ax1)
pylab.plot(t[1:], U)

pylab.xlabel("T")
pylab.ylabel("U")
pylab.legend(["u"])
pylab.grid()
pylab.show()