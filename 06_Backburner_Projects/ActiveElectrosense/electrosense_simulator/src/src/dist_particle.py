# Eliminate hard coded maze
from __future__ import absolute_import

import random
import math
import bisect

from dist_draw import Maze

#-------------------------------------------------------------------
# Electric Field model
tx1 = [6, -2, 0]
tx2 = [-2., 2+6, 0]
rx1 = [0,  0, 0]
rx2 = [6, 2., 0]

def get_dist(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) + (z1 - z2) ** 2 + 0.001

def Ef(x,y,z):
    t = 0.0005
    Vdc = 2000
    # Frequency is 1000Hz
    e1 = Vdc*math.cos(2*math.pi*t*1000)/get_dist(tx1[0], tx1[1], tx1[2], x, y, z)
    e2 = Vdc*math.cos(2*math.pi*t*1000 + math.pi)/ get_dist(tx2[0], tx2[1], tx2[2], x, y, z)
    return e1+e2

def get_r(x, y, z):
    r1 = get_dist(rx1[0], rx1[1], rx1[2], x, y, z)
    r2 = get_dist(rx2[0], rx2[1], rx2[2], x, y, z)
    return ((r1**-1) - (r2**-1))

def dPhi(x,y, z):
    a = 3.8/2.
    chi = 1.
    return (a**3)*chi*Ef(x,y,z)*get_r(x,y,z)

"""
# Smaller maze
maze_data = ( ( 2, 0, 1, 0, 0 ),
              ( 0, 0, 0, 0, 1 ),
              ( 1, 1, 1, 0, 0 ),
              ( 1, 0, 0, 0, 0 ),
              ( 0, 0, 2, 0, 1 ))
"""
# 0 - empty square
# 1 - occupied square
# 2 - occupied square with a beacon at each corner, detectable by the robot


maze_data = ( ( dPhi(-5,  5, 0)/2000, dPhi(-4,  5, 0)/2000, dPhi(-3,  5, 0)/2000, dPhi(-2,  5, 0)/2000, dPhi(-1,  5, 0)/2000, dPhi(0,  5, 0)/2000, dPhi(1,  5, 0)/2000, dPhi(2,  5, 0)/2000, dPhi(3,  5, 0)/2000, dPhi(4,  5, 0)/2000, dPhi(5,  5, 0)/2000 ),
              ( dPhi(-5,  4, 0)/2000, dPhi(-4,  4, 0)/2000, dPhi(-3,  4, 0)/2000, dPhi(-2,  4, 0)/2000, dPhi(-1,  4, 0)/2000, dPhi(0,  4, 0)/2000, dPhi(1,  4, 0)/2000, dPhi(2,  4, 0)/2000, dPhi(3,  4, 0)/2000, dPhi(4,  4, 0)/2000, dPhi(5,  4, 0)/2000 ),
              ( dPhi(-5,  3, 0)/2000, dPhi(-4,  3, 0)/2000, dPhi(-3,  3, 0)/2000, dPhi(-2,  3, 0)/2000, dPhi(-1,  3, 0)/2000, dPhi(0,  3, 0)/2000, dPhi(1,  3, 0)/2000, dPhi(2,  3, 0)/2000, dPhi(3,  3, 0)/2000, dPhi(4,  3, 0)/2000, dPhi(5,  3, 0)/2000 ),
              ( dPhi(-5,  2, 0)/2000, dPhi(-4,  2, 0)/2000, dPhi(-3,  2, 0)/2000, dPhi(-2,  2, 0)/2000, dPhi(-1,  2, 0)/2000, dPhi(0,  2, 0)/2000, dPhi(1,  2, 0)/2000, dPhi(2,  2, 0)/2000, dPhi(3,  2, 0)/2000, dPhi(4,  2, 0)/2000, dPhi(5,  2, 0)/2000 ),
              ( dPhi(-5,  1, 0)/2000, dPhi(-4,  1, 0)/2000, dPhi(-3,  1, 0)/2000, dPhi(-2,  1, 0)/2000, dPhi(-1,  1, 0)/2000, dPhi(0,  1, 0)/2000, dPhi(1,  1, 0)/2000, dPhi(2,  1, 0)/2000, dPhi(3,  1, 0)/2000, dPhi(4,  1, 0)/2000, dPhi(5,  1, 0)/2000 ),
              ( dPhi(-5,  0, 0)/2000, dPhi(-4,  0, 0)/2000, dPhi(-3,  0, 0)/2000, dPhi(-2,  0, 0)/2000, dPhi(-1,  0, 0)/2000, dPhi(0,  0, 0)/2000, dPhi(1,  0, 0)/2000, dPhi(2,  0, 0)/2000, dPhi(3,  0, 0)/2000, dPhi(4,  0, 0)/2000, dPhi(5,  0, 0)/2000 ),
              ( dPhi(-5, -1, 0)/2000, dPhi(-4, -1, 0)/2000, dPhi(-3, -1, 0)/2000, dPhi(-2, -1, 0)/2000, dPhi(-1, -1, 0)/2000, dPhi(0, -1, 0)/2000, dPhi(1, -1, 0)/2000, dPhi(2, -1, 0)/2000, dPhi(3, -1, 0)/2000, dPhi(4, -1, 0)/2000, dPhi(5, -1, 0)/2000 ),
              ( dPhi(-5, -2, 0)/2000, dPhi(-4, -2, 0)/2000, dPhi(-3, -2, 0)/2000, dPhi(-2, -2, 0)/2000, dPhi(-1, -2, 0)/2000, dPhi(0, -2, 0)/2000, dPhi(1, -2, 0)/2000, dPhi(2, -2, 0)/2000, dPhi(3, -2, 0)/2000, dPhi(4, -2, 0)/2000, dPhi(5, -2, 0)/2000 ),
              ( dPhi(-5, -3, 0)/2000, dPhi(-4, -3, 0)/2000, dPhi(-3, -3, 0)/2000, dPhi(-2, -3, 0)/2000, dPhi(-1, -3, 0)/2000, dPhi(0, -3, 0)/2000, dPhi(1, -3, 0)/2000, dPhi(2, -3, 0)/2000, dPhi(3, -3, 0)/2000, dPhi(4, -3, 0)/2000, dPhi(5, -3, 0)/2000 ),
              ( dPhi(-5, -4, 0)/2000, dPhi(-4, -4, 0)/2000, dPhi(-3, -4, 0)/2000, dPhi(-2, -4, 0)/2000, dPhi(-1, -4, 0)/2000, dPhi(0, -4, 0)/2000, dPhi(1, -4, 0)/2000, dPhi(2, -4, 0)/2000, dPhi(3, -4, 0)/2000, dPhi(4, -4, 0)/2000, dPhi(5, -4, 0)/2000 ),
              ( dPhi(-5, -5, 0)/2000, dPhi(-4, -5, 0)/2000, dPhi(-3, -5, 0)/2000, dPhi(-2, -5, 0)/2000, dPhi(-1, -5, 0)/2000, dPhi(0, -5, 0)/2000, dPhi(1, -5, 0)/2000, dPhi(2, -5, 0)/2000, dPhi(3, -5, 0)/2000, dPhi(4, -5, 0)/2000, dPhi(5, -5, 0)/2000 ))

'''
x0 = -5
y0 = -5
for ii in range(10):
    x0 += ii
    for jj in range(10):
        maze_data[ii, jj] = dPhi(x0, y0+jj, 0)/2000.
'''
PARTICLE_COUNT = 3000    # Total number of particles

ROBOT_HAS_COMPASS = True # Does the robot know where north is? If so, it
# makes orientation a lot easier since it knows which direction it is facing.
# If not -- and that is really fascinating -- the particle filter can work
# out its heading too, it just takes more particles and more time. Try this
# with 3000+ particles, it obviously needs lots more hypotheses as a particle
# now has to correctly match not only the position but also the heading.

# ------------------------------------------------------------------------
# Some utility functions

def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]

def add_little_noise(*coords):
    return add_noise(0.02, *coords)

def add_some_noise(*coords):
    return add_noise(0.1, *coords)

# This is just a gaussian kernel I pulled out of my hat, to transform
# values near to robot's measurement => 1, further away => 0
var_xs_ys = 0.1
def w_gauss(a, b):
    error = a - b
    sigma2 = var_xs_ys**0.5
    g = math.e ** -(error ** 2 / (sigma2))
    return g

# ------------------------------------------------------------------------
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
    m_count = 0
    for p in particles:
        if world.distance(p.x, p.y, m_x, m_y) < 1:
            m_count += 1

    return m_x, m_y, m_count > PARTICLE_COUNT * 0.95

# ------------------------------------------------------------------------
class WeightedDistribution(object):
    def __init__(self, state):
        accum = 0.0
        self.state = [p for p in state if p.w > 0]
        self.distribution = []
        for x in self.state:
            accum += x.w
            self.distribution.append(accum)

    def pick(self):
        try:
            return self.state[bisect.bisect_left(self.distribution, random.uniform(0, 1))]
        except IndexError:
            # Happens when all particles are improbable w=0
            return None


# ------------------------------------------------------------------------
class Particle(object):
    def __init__(self, x, y, heading=None, w=1, noisy=False):
        if heading is None:
            heading = random.uniform(0, 360)
        if noisy:
            x, y, heading = add_some_noise(x, y, heading)

        self.x = x
        self.y = y
        self.phi = dPhi(x,y,0)
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

    @classmethod
    def create_random(cls, count, maze):
        return [cls(*maze.random_free_place()) for _ in range(0, count)]

    def read_sensor(self, maze):
        """
        Find distance to nearest beacon.
        """
        return abs(self.phi - expected_phi)
        #return maze.distance_to_nearest_beacon(*self.xy)

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
class Robot(Particle):
    speed = 0.2

    def __init__(self, maze):
        self.rob_pose = (random.uniform(-9, 9), random.uniform(-9, 9))
        super(Robot, self).__init__(self.rob_pose[0], self.rob_pose[1], heading=90) #*maze.random_free_place()
        self.chose_random_direction()
        self.step_count = 0

    def get_pose(self):
        print(self.rob_pose)
        return self.rob_pose

    def chose_random_direction(self):
        heading = random.uniform(0, 360)
        self.h = heading

    def read_sensor(self, maze):
        """
        Robot sensors are noisy and pretty strange,
        it only can measure the distance to the nearest beacon(!)
        and is not very accurate at that too!
        """
        return super(Robot, self).read_sensor(maze)#add_little_noise(super(Robot, self).read_sensor(maze))[0]

    def move(self, maze):
        """
        Move the robot. Note that the movement is stochastic too.
        """
        while True:
            self.step_count += 1
            if self.advance_by(self.speed, noisy=True,
                checker=lambda r, dx, dy: maze.is_free(r.x+dx, r.y+dy)):
                break
            # Bumped into something or too long in same direction,
            # chose random new direction
            self.chose_random_direction()

# ------------------------------------------------------------------------
world = Maze(maze_data)
world.draw()

# initial distribution assigns each particle an equal probability
particles = Particle.create_random(PARTICLE_COUNT, world)
rob = Robot(world)

ii = 0
while True:
    if ii == 1:
        print('Goal Turtle at')
        print(world.rob_pose)
    ii += 1
    expected_phi = dPhi(world.rob_pose[0], world.rob_pose[1], 0)
    # Read rob's sensor
    r_d = rob.read_sensor(world)

    # Update particle weight according to how good every particle matches
    # rob's sensor reading
    for p in particles:
        if world.is_free(*p.xy):
            p_d = p.read_sensor(world)
            p.w = w_gauss(r_d, p_d)
        else:
            p.w = 0

    # ---------- Try to find current best estimate for display ----------
    m_x, m_y, m_confident = compute_mean_point(particles)

    # ---------- Show current state ----------
    world.show_particles(particles)
    world.show_mean(m_x, m_y, m_confident)
    world.show_robot(rob)

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
            new_particle = Particle.create_random(1, world)[0]
        else:
            new_particle = Particle(p.x, p.y,
                    heading=rob.h if ROBOT_HAS_COMPASS else p.h,
                    noisy=True)
        new_particles.append(new_particle)

    particles = new_particles

    # # ---------- Move things ----------
    # old_heading = rob.h
    # rob.move(world)
    # d_h = rob.h - old_heading

    # # Move particles according to my belief of movement (this may
    # # be different than the real movement, but it's all I got)
    # for p in particles:
    #     p.h += d_h # in case robot changed heading, swirl particle heading too
    #     p.advance_by(rob.speed)