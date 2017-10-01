# The particle filter, prediciton only.
#
# slam_08_a_particle_prediciton.
# Claus Brenner, 04.01.2013
from lego_robot import *
from math import sin, cos, pi, sqrt
import random

class ParticleFilter:
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

    # State transition. This is exactly the same method as in the Kalman filter.
    @staticmethod
    def g(state, control, w):
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

        return (g1, g2, g3)

    def predict(self, control):
        """The prediction step of the particle filter."""
        left, right = control

        # --->>> Put your code here.

        # Compute left and right variance.
        # alpha_1 is self.control_motion_factor.
        # alpha_2 is self.control_turn_factor.
        sigma_lt = sqrt((self.control_motion_factor * left)**2 + (self.control_turn_factor*(left - right))**2)
        sigma_rt = sqrt((self.control_motion_factor * right)**2 + (self.control_turn_factor*(left - right))**2)

        # Then, do a loop over all self.particles and construct a new
        # list of particles.
        new_particles = []
        for p in self.particles:
            # For sampling, use random.gauss(mu, sigma). (Note sigma in this call
            # is the standard deviation, not the variance.)
            lt_prime = random.gauss(left,  sigma_lt)
            rt_prime = random.gauss(right, sigma_rt)
            new_particles.append(ParticleFilter.g(p, (lt_prime, rt_prime), self.robot_width))
            
        # In the end, assign the new list of particles to self.particles.
        self.particles = new_particles


    def print_particles(self, file_desc):
        """Prints particles to given file_desc output."""
        if not self.particles:
            return
        print >> file_desc, "PA",
        for p in self.particles:
            print >> file_desc, "%.0f %.0f %.3f" % p,
        print >> file_desc


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.

    # Generate initial particles. Each particle is (x, y, theta).
    number_of_particles = 300
    # Measured start position.
    # This is the origin point for the laser. See SLAM-A-Getting_started/11-Getting_started-Files/slam_02_b_filter_motor_file_question.py
    measured_state = (1850.0, 1897.0, 213.0 / 180.0 * pi)
    # We have to obtain the origin point for the center of the robot.
    measured_state = (measured_state[0] - scanner_displacement*cos(measured_state[2]), measured_state[1] - scanner_displacement*sin(measured_state[2]), measured_state[2])

    standard_deviations = (100.0, 100.0, 10.0 / 180.0 * pi)
    initial_particles = []
    for i in xrange(number_of_particles):
        initial_particles.append(tuple([
            random.gauss(measured_state[j], standard_deviations[j])
            for j in xrange(3)]))

    # Setup filter.
    pf = ParticleFilter(initial_particles,
                        robot_width, scanner_displacement,
                        control_motion_factor, control_turn_factor)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records.
    # This is the particle filter loop, with prediction and correction.
    f = open("particle_filter_predicted.txt", "w")
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        control = map(lambda x: x * ticks_to_mm, logfile.motor_ticks[i])
        pf.predict(control)

        # Output particles.
        pf.print_particles(f)

    f.close()
