import numpy as np
import trep
import sactrep
import pylab

# set mass, length, and gravity:
m = 1.0; l = 1.0; g = 9.81; mc = 1.0;

# define initial config and velocity
q0 = np.array([0, np.pi]) # q = [x_cart, theta]
dq0 = np.array([0, 0]) # dq = [xdot, thetadot]

# define time parameters:
dt = 0.0167
tf = 10.0

# create system
system = trep.System()
# define frames
frames = [
    trep.tx("x_cart", name="CartFrame", mass=mc), [
        trep.rz("theta", name="PendulumBase"), [
            trep.ty(l, name="Pendulum", mass=m)]]]
# add frames to system
system.import_frames(frames)
# add gravity potential
trep.potentials.Gravity(system, (0,-g,0))
# add a horizontal force on the cart
trep.forces.ConfigForce(system, "x_cart", "cart_force")


#############
# SAC STUFF #
#############

def proj_func(x):
    x[1] = np.fmod(x[1]+np.pi, 2.0*np.pi)
    if(x[1] < 0):
        x[1] = x[1]+2.0*np.pi
    x[1] = x[1] - np.pi
    ##
    # Create a iLQR feedback

sacsys = sactrep.Sac(system)

sacsys.T = 1.2
sacsys.lam = -5
sacsys.maxdt = 0.2
sacsys.ts = dt
sacsys.usat = [[10, -10]]
sacsys.calc_tm = 0.0
sacsys.u2search = False
sacsys.Q = np.diag([100,200,50,0]) # x,th,xd,thd
sacsys.P = 0*np.diag([0,0,0,0])
sacsys.R = 0.3*np.identity(1)

sacsys.set_proj_func(proj_func)

# set initial conditions:
system.q = q0
system.dq = dq0

# init SAC:
sacsys.init()

# run loop:
q = np.hstack((system.q[0], system.q[1],
               system.dq[0], system.dq[1]))
u = np.array([system.u])
while sacsys.time < tf:
    sacsys.step()
    q = np.vstack((q, np.hstack((system.q[0], system.q[1],
                                 system.dq[0], system.dq[1]))))
    u = np.vstack((u, system.u))
    if np.abs(sacsys.time%1)<dt:
        print "time = ",sacsys.time

#sacsys.save()

nn = np.shape(np.array(u))[0]

fig = pylab.figure()
ax = fig.gca()
ax.set_xticks(np.arange(0, 10, 0.5))
ax.set_yticks(np.arange(-11., 11., 1.))
pylab.plot(np.linspace(0,tf, num=nn), zip(*q)[1])
pylab.plot(np.linspace(0,tf, num=nn), zip(*u)[0])
pylab.plot(np.linspace(0,tf, num=nn), np.pi*np.ones(nn))
pylab.plot(np.linspace(0,tf, num=nn), 0.*np.pi*np.ones(nn))
pylab.grid()
#pylab.plot(np.linspace(0,tf, num=nn), u)
#pylab.legend(["x","theta", "x_dot", "th_dot"])
pylab.legend(["theta", "theta2", "pi"])
pylab.show()
#sacsys.get_final_traj()
#sacsys.save()
# # Visualize the system in action
# visual.visualize_3d([ visual.VisualItem3D(system, np.linspace(0,tf, num=nn) , q) ])

# Create a visualization of the scissor lift.
# visual.visualize_2d([visual.VisualItem2D(system, np.linspace(0,tf, num=nn), q, plane='XZ')])

#nutrisystem