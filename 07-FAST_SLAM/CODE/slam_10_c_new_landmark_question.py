# FastSLAM.
# Particle member functions for initializing a new landmark.
#
# slam_10_c_new_landmark
# Claus Brenner, 19.02.2013
from lego_robot import *
from math import sin, cos, pi, atan2, sqrt, exp
import numpy as np


class Particle:
    def __init__(self, pose):
        self.pose = pose
        self.landmark_positions = []
        self.landmark_covariances = []

    def number_of_landmarks(self):
        """Utility: return current number of landmarks in this particle."""
        return len(self.landmark_positions)

    @staticmethod
    def dh_dlandmark(state, landmark, scanner_displacement):
        """Derivative with respect to the landmark coordinates. This is related
           to the dh_dstate function we used earlier (it is:
           -dh_dstate[0:2,0:2])."""
        theta = state[2]
        cost, sint = cos(theta), sin(theta)
        dx = landmark[0] - (state[0] + scanner_displacement * cost)
        dy = landmark[1] - (state[1] + scanner_displacement * sint)
        q = dx * dx + dy * dy
        sqrtq = sqrt(q)
        dr_dmx = dx / sqrtq
        dr_dmy = dy / sqrtq
        dalpha_dmx = -dy / q
        dalpha_dmy =  dx / q

        return np.array([[dr_dmx, dr_dmy],
                         [dalpha_dmx, dalpha_dmy]])

    def initialize_new_landmark(self, measurement_in_scanner_system,
                                Qt_measurement_covariance,
                                scanner_displacement):
        """Given a (x, y) measurement in the scanner's system, initializes a
           new landmark and its covariance."""
        scanner_pose = (self.pose[0] + cos(self.pose[2]) * scanner_displacement,
                        self.pose[1] + sin(self.pose[2]) * scanner_displacement,
                        self.pose[2])
        # Hints:
        # - LegoLogfile.scanner_to_world() (from lego_robot.py) will return
        #   the world coordinate, given the scanner pose and the coordinate in
        #   the scanner's system.
        xlm, ylm = LegoLogfile.scanner_to_world(scanner_pose, measurement_in_scanner_system)
        self.landmark_positions.append(np.array([xlm, ylm]))  # Replace this.
        # - H is obtained from dh_dlandmark()
        H = Particle.dh_dlandmark(self.pose, (xlm, ylm), scanner_displacement)
        # - Use np.linalg.inv(A) to invert matrix A
        # - As usual, np.dot(A,B) is the matrix product of A and B.
        Sigma_lm = np.dot(np.linalg.inv(H), np.dot(Qt_measurement_covariance, np.linalg.inv(H).T))
        self.landmark_covariances.append(Sigma_lm)

if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0

    # Filter constants.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.
    Qt_measurement_covariance = \
        np.diag([measurement_distance_stddev**2,
                 measurement_angle_stddev**2])

    # Define a particle: position (x, y) and orientation.
    p = Particle(np.array([-scanner_displacement, 0.0, 0.0]))

    # Add a landmark along the x axis.
    measurement_in_scanner_system = (1000.0, 0.0)
    p.initialize_new_landmark(measurement_in_scanner_system,
                              Qt_measurement_covariance,
                              scanner_displacement)
    # Add another landmark at twice the distance.
    measurement_in_scanner_system = (2000.0, 0.0)
    p.initialize_new_landmark(measurement_in_scanner_system,
                              Qt_measurement_covariance,
                              scanner_displacement)
    # Add another landmark at the distance of the first landmark, but at
    # a bearing angle of 45 degrees.
    measurement_in_scanner_system = np.array([1000.0, 1000.0]) / sqrt(2)
    p.initialize_new_landmark(measurement_in_scanner_system,
                              Qt_measurement_covariance,
                              scanner_displacement)

    measurement_in_scanner_system = np.array([1500.0*cos(-pi/3), 1500.0*sin(-pi/3)])
    p.initialize_new_landmark(measurement_in_scanner_system,
                              Qt_measurement_covariance,
                              scanner_displacement)
    # Print all landmarks.
    for i in xrange(p.number_of_landmarks()):
        print "Landmark", i, "----------"
        print " Position:", p.landmark_positions[i]
        print " Landmark covariance:\n ", p.landmark_covariances[i]
        print " This corresponds to the error ellipse:"
        eigenvals, eigenvects = np.linalg.eig(p.landmark_covariances[i])
        print(eigenvects)
        print(eigenvals)
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        print " Angle [deg]:", angle / pi * 180.0
        print " Axis 1:", sqrt(eigenvals[0])
        print " Axis 2:", sqrt(eigenvals[1])
