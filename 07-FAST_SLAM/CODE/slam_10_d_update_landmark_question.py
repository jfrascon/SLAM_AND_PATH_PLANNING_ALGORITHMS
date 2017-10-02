# FastSLAM.
# Particle member function for updating a landmark.
#
# slam_10_d_update_landmark
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
    def h(state, landmark, scanner_displacement):
        """Measurement function. Takes a (x, y, theta) state and a (x, y)
           landmark, and returns the corresponding (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi
        return np.array([r, alpha])

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

    def h_expected_measurement_for_landmark(self, landmark_number,
                                            scanner_displacement):
        # Note: This is just one line of code!
        # Hints:
        # - the static function h() computes the desired value
        # - the state is the robot's pose
        # - the landmark is taken from self.landmark_positions.
        return Particle.h(self.pose, self.landmark_positions[landmark_number], scanner_displacement)

    def H_Ql_jacobian_and_measurement_covariance_for_landmark(
        self, landmark_number, Qt_measurement_covariance, scanner_displacement):
        """Computes Jacobian H of measurement function at the particle's
           position and the landmark given by landmark_number. Also computes the
           measurement covariance matrix."""
        # Hints:
        # - H is computed using dh_dlandmark.
        # - To compute Ql, you will need the product of two matrices,
        #   which is np.dot(A, B).
        H = Particle.dh_dlandmark(self.pose, self.landmark_positions[landmark_number], scanner_displacement)
        Ql = np.dot(H, np.dot(self.landmark_covariances[landmark_number], H.T)) + Qt_measurement_covariance
        return (H, Ql)

    def update_landmark(self, landmark_number, measurement,
                        Qt_measurement_covariance, scanner_displacement):
        """Update a landmark's estimated position and covariance."""
        # Hints:
        # - H and Ql can be computed using
        #   H_Ql_jacobian_and_measurement_covariance_for_landmark()
        H, Ql = self.H_Ql_jacobian_and_measurement_covariance_for_landmark(landmark_number, Qt_measurement_covariance, scanner_displacement)
        # - Use np.linalg.inv(A) to compute the inverse of A
        K = np.dot(self.landmark_covariances[landmark_number], np.dot(H.T, np.linalg.inv(Ql)))
        # - Delta z is measurement minus expected measurement, also called innovation.
        # - Expected measurement can be computed using h_expected_measurement_for_landmark()
        innovation = measurement - self.h_expected_measurement_for_landmark(landmark_number, scanner_displacement)
        xlm, ylm = self.landmark_positions[landmark_number] + np.dot(K, innovation)
        sigma_lm = np.dot((np.eye(2) - np.dot(K, H)), self.landmark_covariances[landmark_number])
        # - Remember to update landmark_positions[landmark_number] as well
        #   as landmark_covariances[landmark_number].
        self.landmark_positions[landmark_number] = (xlm, ylm)
        self.landmark_covariances[landmark_number] = sigma_lm

def insert_landmarks(particle):
    """Insert the landmarks from the slam_10_c_new_landmark exercise."""
    positions = [
        np.array([1000.0, 0.0]),
        np.array([2000.0, 0.0]),
        np.array([707.10678118654744, 707.10678118654744])
        ]
    covariances = [
        np.array([[ 40000., 0.],
                  [ 0., 68538.91945201]]),
        np.array([[ 40000., 0.],
                  [ 0., 274155.67780804]]),
        np.array([[ 54269.459726, -14269.459726],
                  [ -14269.459726,54269.459726]])
        ]
    particle.landmark_positions.extend(positions)
    particle.landmark_covariances.extend(covariances)

def print_landmarks(particle):
    for i in xrange(particle.number_of_landmarks()):
        print "Landmark", i, "----------"
        print " Position:", particle.landmark_positions[i]
        print " Error ellipse:"
        eigenvals, eigenvects = np.linalg.eig(particle.landmark_covariances[i])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        print "  Angle [deg]:", angle / pi * 180.0
        print "  Axis 1:", sqrt(eigenvals[0])
        print "  Axis 2:", sqrt(eigenvals[1])


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

    # Insert the landmarks from the slam_10_c_new_landmark exercise.
    insert_landmarks(p)

    # Print all landmarks (before update).
    print "Landmarks - before update:"
    print_landmarks(p)

    # Measure first landmark.
    # Assume we would measure the exact distance (1000) and bearing (0.0).
    measurement = np.array([1000.0, 0.0])
    p.update_landmark(0, measurement, Qt_measurement_covariance,
                      scanner_displacement)

    # Measure second landmark.
    # Assume we would measure a slightly different range.
    measurement = np.array([2000.0 + 100.0, 0.0])
    p.update_landmark(1, measurement, Qt_measurement_covariance,
                      scanner_displacement)

    # Print all landmarks (after update).
    print "\nLandmarks - after update:"
    print_landmarks(p)
