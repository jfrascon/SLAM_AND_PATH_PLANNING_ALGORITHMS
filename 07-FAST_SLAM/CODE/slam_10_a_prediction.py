# FastSLAM, first version: prediciton only.
#
# slam_10_a_prediction
# Claus Brenner, 17.02.2013
from lego_robot import *
from slam_g_library import get_mean, get_error_ellipse_and_heading_variance,\
     print_particles
from math import sin, cos, pi, sqrt
import copy
import random
import numpy as np


class Particle:
    def __init__(self, pose):
        self.pose = pose

    @staticmethod
    def g(state, control, w):
        """State transition. This is exactly the same method as in the Kalman
           filter."""
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta
        return np.array([g1, g2, g3])

    def move(self, left, right, w):
        """Given left, right control and robot width, move the robot."""
        self.pose = self.g(self.pose, (left, right), w)


class FastSLAM:
    def __init__(self, initial_particles,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor):
        # The particles.
        self.particles = initial_particles

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor

    def predict(self, control):
        """The prediction step of the particle filter."""
        left, right = control
        left_std  = sqrt((self.control_motion_factor * left)**2 +\
                        (self.control_turn_factor * (left-right))**2)
        right_std = sqrt((self.control_motion_factor * right)**2 +\
                         (self.control_turn_factor * (left-right))**2)
        # Modify list of particles: for each particle, predict its new position.
        for p in self.particles:
            l = random.gauss(left, left_std)
            r = random.gauss(right, right_std)
            p.move(l, r, self.robot_width)


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.

    # Generate initial particles. Each particle is (x, y, theta).
    number_of_particles = 25
    start_state = np.array([500.0, 0.0, 45.0 / 180.0 * pi])
    # The copy.copy method is used but it is a normal copy, it is the reference
    # of the newly created Particle object is copied into the initial_particles list.
    initial_particles = [copy.copy(Particle(start_state))
                         for _ in xrange(number_of_particles)]

    # Setup filter.
    fs = FastSLAM(initial_particles,
                  robot_width, scanner_displacement,
                  control_motion_factor, control_turn_factor)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records.
    # This is the FastSLAM filter loop, prediction only.
    f = open("fast_slam_prediction.txt", "w")
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        control = map(lambda x: x * ticks_to_mm, logfile.motor_ticks[i])
        fs.predict(control)

        # Output particles.
        print_particles(fs.particles, f)

        # Output state estimated from all particles.
        mean = get_mean(fs.particles)
        print >> f, "F %.0f %.0f %.3f" %\
              (mean[0] + scanner_displacement * cos(mean[2]),
               mean[1] + scanner_displacement * sin(mean[2]),
               mean[2])

        # Output error ellipse and standard deviation of heading.
        errors = get_error_ellipse_and_heading_variance(fs.particles, mean)
        print >> f, "E %.3f %.0f %.0f %.3f" % errors

    f.close()
