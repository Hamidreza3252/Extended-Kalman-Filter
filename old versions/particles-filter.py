# 1
# Once again, your robot starts at 30, 50, heading north (pi/2), then turns clockwise 
# by pi/2, moves 15 meters, senses, then turns clockwise by pi/2 again, moves  10 m, then senses again.

# 2
# Now add noise to your robot as follows: forward_noise = 5.0, turn_noise = 0.1, sense_noise = 5.0. 
# Your program should print out the result of your two sense measurements.

# 3
# Now we want to create particles, p[i] = robot(). In this assignment, write code that will assign 1000 such particles to a list.

# 4
# Your program should print out the length of your list (don't cheat by making an arbitrary list of 1000 elements!)

# 5
# Now we want to simulate robot motion with our particles. Each particle should turn by 0.1 and then move by 5. 

# 6
# Now we want to give weight to our particles. This program will print a list of 1000 particle weights. 

# 7
# In this exercise, try to write a program that # will resample particles according to their weights. Particles with higher weights should be sampled
# more frequently (in proportion to their weight). 

# Don't modify the code below. Please enter your code at the bottom. 

from math import *
import random
import numpy as np

landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0


class Robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0
        self.turn_noise    = 0.0
        self.sense_noise   = 0.0
    
    def set(self, new_x, new_y, new_orientation):
        if (new_x < 0 or new_x >= world_size):
            raise ValueError("X coordinate out of bound")
        if new_y < 0 or new_y >= world_size:
            raise ValueError("Y coordinate out of bound")
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError("Orientation must be in [0..2pi]")
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    
    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise)
        self.turn_noise    = float(new_t_noise)
        self.sense_noise   = float(new_s_noise)
    
    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z
    
    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError("Robot cant move backwards")
        
        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi
        
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size    # cyclic truncate
        y %= world_size
        
        # set particle
        # Hamid 
        # res = Robot()
        # res.set(x, y, orientation)
        # res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)

        self.set(x, y, orientation)

        # return res
    
    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
    def measurement_prob(self, measurement):
        
        # calculates how likely a measurement should be
        
        prob = 1.0
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob
    
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))


def eval(r, p):
    sum = 0.0
    for i in range(len(p)): # calculate mean error
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p))


####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

"""
pi_2 = pi / 2.0

robot = Robot()
robot.set_noise(5.0, 0.1, 5.0)

robot.set(30.0, 50.0, pi_2)
robot.move(-pi_2, 15.0)
print(robot.sense())

robot.move(-pi_2, 10.0)
print(robot.sense())
"""

robot = Robot()
robot.move(0.1, 5.0)
measurements = robot.sense()

# print(measurements)

particleCount = 100

# particles = [Robot() for _ in range(particleCount)]
particles = particleCount*[Robot()]
weights = particleCount*[0.0]

for i, particle in enumerate(particles):

    particle.set_noise(0.05, 0.05, 5.0)
    particle.move(0.1, 5.0)

    weights[i] = particle.measurement_prob(measurements)

# allSenses = [particle.sense() for particle in particles]
# weights = [particle.measurement_prob(senses) for particle, senses in zip(particles, allSenses)]
# weights = np.asarray([particle.measurement_prob(measurements) for particle in particles])

weights /= np.sum(np.asarray(weights))

particles = np.random.choice(particles, replace=True, p=weights)

print(weights)

