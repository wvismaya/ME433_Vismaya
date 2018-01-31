## Test particle filter
# Eliminate hard coded maze
from __future__ import absolute_import

import random
import math
import bisect

#-------------------------------------------------------------------
# Electric Field model
tx1 = [-5.,  0., 0.]
tx2 = [ 5.,  0., 0.]
rx1 = [ 0., -5., 0.]
rx2 = [ 0.,  5., 0.]

def shift_origin(x, y):
	global tx1
	tx1 = [x-5.,  y+0., 0.]
	global tx2
	tx2 = [x+5.,  y+0., 0.]
	global rx1
	rx1 = [x,  y-5., 0.]
	global rx2
	rx2 = [x,  y+5., 0.]

def get_dist(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2) #+ 0.00001

def Ef(x,y,z):
    t = 0.0005
    Vdc = 2000.
    # Frequency is 1000Hz
    e1 = Vdc*math.cos(2*math.pi*t*1000)/get_dist(tx1[0], tx1[1], tx1[2], x, y, z)
    e2 = Vdc*math.cos(2*math.pi*t*1000 + math.pi)/ get_dist(tx2[0], tx2[1], tx2[2], x, y, z)
    return e1+e2

def get_r(x, y, z):
    r1 = get_dist(rx1[0], rx1[1], rx1[2], x, y, z)**2
    r2 = get_dist(rx2[0], rx2[1], rx2[2], x, y, z)**2
    return ((r1**-1) - (r2**-1))

def dPhi(x,y, z):
    a = 3.8/2.
    chi = 1.
    return (a**3)*chi*Ef(x,y,0)*get_r(x,y,0)


PARTICLE_COUNT = 5000    # Total number of particles

# ------------------------------------------------------------------------
# Some utility functions

def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]

def add_little_noise(*coords):
    return add_noise(0.02, *coords)

def add_some_noise(*coords):
    return add_noise(0.1, *coords)

var_xs_ys = 1.
def w_gauss(a, b):
    error = abs(a-b)
    sigma1 = var_xs_ys**0.5
    g = math.e ** -(error ** 2 / (sigma1))
    #print(g)
    return g

# ------------------------------------------------------------------------
def distance(x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def compute_mean_point(particles):
    """
    Compute the mean for all particles that have a reasonably good weight.
    This is not part of the particle filter algorithm but rather an
    addition to show the "best belief" for current position.
    """

    m_x, m_y, m_count = 0, 0, 0
    for p in particles:
        m_count += p.w
        m_x += p.x * p.w
        m_y += p.y * p.w

    if m_count == 0:
        return -1, -1, False

    m_x /= m_count
    m_y /= m_count

    # Now compute how good that mean is -- check how many particles
    # actually are in the immediate vicinity
    m_count = 0.
    for p in particles:
        if distance(p.x, p.y, m_x, m_y) < 1:
            m_count += 1

    return m_x, m_y, m_count > PARTICLE_COUNT * 0.95

# ------------------------------------------------------------------------
class WeightedDistribution(object):
    def __init__(self, state):
        accum = 0.0
        self.state = [p for p in state if p.w > 0.]
        self.distribution = []
        for x in self.state:
            accum += x.w
            self.distribution.append(accum)

    def pick(self):
        try:
            return self.state[bisect.bisect_left(self.distribution, random.uniform(0.4, 1))]
        except IndexError:
            # Happens when all particles are improbable w=0
            return None

bx = 7.
by = 7.

def set_bounds(x1, y1):
	global bx
	bx -= x1
	global by
	by -= y1

def random_place():
	x = random.uniform(-bx, bx)
	y = random.uniform(-by, by)
	return x, y

def random_free_place():
	#while True:
	x, y = random_place()
	#if self.is_free(x, y):
	return x, y

# ------------------------------------------------------------------------
class Particle(object):
    def __init__(self, x, y, heading=None, w=1, noisy=False):
        if heading is None:
            heading = random.uniform(0, 360)
        if noisy:
            x, y, heading = add_some_noise(x, y, heading)

        self.x = x
        self.y = y
        self.phi = dPhi(self.x, self.y,0)
        self.h = heading
        self.w = w

    def __repr__(self):
        return "(%f, %f, w=%f)" % (self.x, self.y, self.w)

    @property
    def get_phi(self):
        return self.phi

    @property
    def xy(self):
        return self.x, self.y

    @property
    def xyh(self):
        return self.x, self.y, self.h

    @property
    def set_xyh(self, x, y, h):
        self.x = x
        self.y = y
        self.h = h

    @classmethod
    def create_random(cls, count):
        return [cls(*random_free_place()) for _ in range(0, count)]

    def read_sensor(self):
        """
        Find distance to nearest beacon.
        """
        return self.phi
        #return maze.distance_to_nearest_beacon(*self.xy)

    def shift_xyz(self, x, y):
    	self.x = self.x - x
    	self.y = self.y - y

    def advance_by(self, speed, checker=None, noisy=False):
        h = self.h
        if noisy:
            speed, h = add_little_noise(speed, h)
            h += random.uniform(-3, 3) # needs more noise to disperse better
        r = math.radians(h)
        dx = math.sin(r) * speed
        dy = math.cos(r) * speed
        if checker is None or checker(self, dx, dy):
            self.move_by(dx, dy)
            return True
        return False

    def move_by(self, x, y):
        self.x += x
        self.y += y

# ------------------------------------------------------------------------
# MAIN CODE
# ------------------------------------------------------------------------

shift_origin(0,0)

test_pos = [random.uniform(-bx, bx), random.uniform(-by, by)]
# initial distribution assigns each particle an equal probability
particles = Particle.create_random(PARTICLE_COUNT)

ii = 0
m_phi = 10000.
expected_phi = dPhi(test_pos[0], test_pos[1], 0)
# Read rob's sensor
r_d = expected_phi

print('Goal Turtle at'), test_pos

# Particle filter converge #1
timeout_val = 1500.
ii = 0.

p_x = 0
p_y = 0
#( abs(m_phi - expected_phi) > 0.005)
m_confident = False

while (abs(m_phi - expected_phi) > 0.5 and (ii < timeout_val)):

	# Update particle weight according to how good every particle matches sensor reading
    for p in particles:
        #if world.is_free(*p.xy):
        p_d = p.read_sensor()
        p.w = w_gauss(r_d, p_d)
        
    # ---------- Try to find current best estimate for display ----------
    m_x, m_y, m_confident = compute_mean_point(particles)
    if(m_confident):
    	m_phi = dPhi(m_x, m_y, 0)
    #print([m_x, m_y, m_confident])
    
    # ---------- Shuffle particles ----------
    new_particles = []

    # Normalise weights
    nu = sum(p.w for p in particles)
    if nu:
        for p in particles:
            p.w = p.w / nu

    # create a weighted distribution, for fast picking
    dist = WeightedDistribution(particles)

    for _ in particles:
        p = dist.pick()
        if p is None:  # No pick b/c all totally improbable
            new_particle = Particle.create_random(1)[0]
        else:
            new_particle = Particle(p.x, p.y,
                    heading= p.h,
                    noisy=False)
        new_particles.append(new_particle)

    particles = new_particles
           
    #expected_phi = dPhi(test_pos[0], test_pos[1], 0)
    ii += 1
    # ---------- Try to find current best estimate for display ----------
    #m_x, m_y, m_confident = compute_mean_point(new_particles)

    #for p in particles:
    #	p.shift_xyz(-m_x, -m_y)
    # Shift Electrodes
    #shift_origin(m_x, m_y)
    #test_pos[0]-=m_x
    #test_pos[1]-=m_y
    #expected_phi = dPhi(test_pos[0], test_pos[1], 0)
p_x = m_x
p_y = m_y

print([p_x, p_y])
print(ii/timeout_val)

#shift_origin(p_x, p_y)

    # # ---------- Move things ----------
    # old_heading = rob.h
    # rob.move(world)
    # d_h = rob.h - old_heading

    # # Move particles according to my belief of movement (this may
    # # be different than the real movement, but it's all I got)
    # for p in particles:
    #     p.h += d_h # in case robot changed heading, swirl particle heading too
    #     p.advance_by(rob.speed)