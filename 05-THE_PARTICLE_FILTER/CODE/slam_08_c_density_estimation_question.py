# The particle filter, prediciton and correction.
# In addition to the filtered particles, the density is estimated. In this
# simple case, the mean position and heading is computed from all particles.
#
# slam_08_c_density_estimation.
# Claus Brenner, 04.01.2013
from lego_robot import *
from slam_e_library import get_cylinders_from_scan, assign_cylinders
from math import sin, cos, pi, atan2, sqrt
import random
from scipy.stats import norm as normal_dist


class ParticleFilter:
    def __init__(self, initial_particles,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev):
        # The particles.
        self.particles = initial_particles

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

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

    # Measurement. This is exactly the same method as in the Kalman filter.
    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           corresponding (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi
        return (r, alpha)

    def probability_of_measurement(self, measurement, predicted_measurement):
        """Given a measurement and a predicted measurement, computes
           probability."""
        # Compute differences to real measurements.

        # --->>> Compute difference in distance and bearing angle.
        # Important: make sure the angle difference works correctly and does
        # not return values offset by 2 pi or - 2 pi.
        weight = 1.0
        for i in xrange(len(measurement)):
            delta_d =  measurement[i][0] - predicted_measurement[i][0]
            delta_alpha = (((measurement[i][1] - predicted_measurement[i][1]) + pi) % (2*pi)) - pi
            # You may use the following Gaussian PDF function:
            # scipy.stats.norm.pdf(x, mu, sigma). With the import in the header,
            # this is normal_dist.pdf(x, mu, sigma).
            # Note that the two parameters sigma_d and sigma_alpha discussed
            # in the lecture are self.measurement_distance_stddev and
            # self.measurement_angle_stddev.
            weight *= normal_dist.pdf(delta_d, 0, self.measurement_distance_stddev) * normal_dist.pdf(delta_alpha, 0, self.measurement_angle_stddev)

        return weight

    def compute_weights(self, cylinders, landmarks):
        """Computes one weight for each particle, returns list of weights."""
        weights = []
        for p in self.particles:
            # Get list of tuples:
            # [ ((range_0, bearing_0), (landmark_x, landmark_y)), ... ]
            assignment = assign_cylinders(cylinders, p,
                self.scanner_displacement, landmarks)

            # --->>> Insert code to compute weight for particle p here.
            # This will require a loop over all (measurement, landmark)
            # in assignment. Append weight to the list of weights.
            measurement = []
            predicted_measurement = []

            # zt_component => measurement to a specific detected cylinder in the scan from particle p.
            for zt_component, landmark in assignment:
                measurement.append(zt_component)
                predicted_measurement.append(ParticleFilter.h(p, landmark, self.scanner_displacement))

            weights.append(self.probability_of_measurement(measurement, predicted_measurement))

        return weights

    def resample(self, weights):
        """Return a list of particles which have been resampled, proportional to the given weights."""
        # --->>> Insert your code here.
        # You may implement the 'resampling wheel' algorithm
        # described in the lecture.
        max_weight = max(weights)
        index = random.randint(0, len(weights)-1)
        offset = 0
        new_particles = []
        for i in xrange(len(weights)):
            offset += random.uniform(0, 2*max_weight)
            while offset > weights[index]:
                offset -= weights[index]
                index += 1
                if index == len(weights): # we can also use the modulus operator in the previous sum
                    index = 0
            new_particles.append(self.particles[index])
        return new_particles

    def correct(self, cylinders, landmarks):
        """The correction step of the particle filter."""
        # First compute all weights.
        weights = self.compute_weights(cylinders, landmarks)
        # Then resample, based on the weight array.
        self.particles = self.resample(weights)

    def print_particles(self, file_desc):
        """Prints particles to given file_desc output."""
        if not self.particles:
            return
        print >> file_desc, "PA",
        for p in self.particles:
            print >> file_desc, "%.0f %.0f %.3f" % p,
        print >> file_desc

    def get_mean(self):
        """Compute mean position and heading from all particles."""

        # --->>> This is the new code you'll have to implement.
        # Return a tuple: (mean_x, mean_y, mean_heading).
        x_accum = 0
        y_accum = 0
        ux = 0
        uy = 0
        for p in self.particles:
            x_accum += p[0]
            y_accum += p[1]
            ux += cos(p[2])
            uy += sin(p[2])

        return (x_accum/len(self.particles), y_accum/len(self.particles), atan2(uy, ux))


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

    # Generate initial particles. Each particle is (x, y, theta).
    number_of_particles = 50
    measured_state = (1850.0, 1897.0, 213.0 / 180.0 * pi)
    standard_deviations = (100.0, 100.0, 10.0 / 180.0 * pi)
    initial_particles = []
    for i in xrange(number_of_particles):
        initial_particles.append(tuple([
            random.gauss(measured_state[j], standard_deviations[j])
            for j in xrange(3)]))

    # Setup filter.
    pf = ParticleFilter(initial_particles,
                        robot_width, scanner_displacement,
                        control_motion_factor, control_turn_factor,
                        measurement_distance_stddev,
                        measurement_angle_stddev)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    # Loop over all motor tick records.
    # This is the particle filter loop, with prediction and correction.
    f = open("particle_filter_mean.txt", "w")
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        control = map(lambda x: x * ticks_to_mm, logfile.motor_ticks[i])
        pf.predict(control)

        # Correction.
        cylinders = get_cylinders_from_scan(logfile.scan_data[i], depth_jump,
            minimum_valid_distance, cylinder_offset)
        pf.correct(cylinders, reference_cylinders)

        # Output particles.
        pf.print_particles(f)

        # Output state estimated from all particles.
        mean = pf.get_mean()
        print >> f, "F %.0f %.0f %.3f" %\
              (mean[0] + scanner_displacement * cos(mean[2]),
               mean[1] + scanner_displacement * sin(mean[2]),
               mean[2])

    f.close()
