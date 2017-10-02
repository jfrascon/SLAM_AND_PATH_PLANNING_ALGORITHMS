# FastSLAM.
# Particle member functions for correspondence likelihood.
#
# slam_10_b_correspondence_likelihood
# Claus Brenner, 18.02.2013
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
        """Returns the expected distance and bearing measurement for a given
           landmark number and the pose of this particle."""
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

    def wl_likelihood_of_correspondence(self, measurement,
                                        landmark_number,
                                        Qt_measurement_covariance,
                                        scanner_displacement):
        """For a given measurement and landmark_number, returns the likelihood
           that the measurement corresponds to the landmark."""
        # Hints:
        # - You will need delta_z, which is the measurement minus the
        #   expected_measurement_for_landmark()
        # - Ql is obtained using a call to
        #   H_Ql_jacobian_and_measurement_covariance_for_landmark(). You
        #   will only need Ql, not H
        # - np.linalg.det(A) computes the determinant of A
        # - np.dot() does not distinguish between row and column vectors.
        delta_z = measurement - self.h_expected_measurement_for_landmark(landmark_number, scanner_displacement)
        # I called Qtk to what the teacher called Ql
        H, Ql = self.H_Ql_jacobian_and_measurement_covariance_for_landmark(landmark_number, Qt_measurement_covariance, scanner_displacement)
        return (1/(2*pi*sqrt(np.linalg.det(Ql)))) * exp(-0.5*np.dot(delta_z.T, np.dot(np.linalg.inv(Ql), delta_z)))

    def compute_correspondence_likelihoods(self, measurement,
                                           number_of_landmarks,
                                           Qt_measurement_covariance,
                                           scanner_displacement):
        """For a given measurement, returns a list of all correspondence
           likelihoods (from index 0 to number_of_landmarks-1)."""
        likelihoods = []
        for i in xrange(number_of_landmarks):
            likelihoods.append(
                self.wl_likelihood_of_correspondence(
                    measurement, i, Qt_measurement_covariance,
                    scanner_displacement))
        return likelihoods


def add_landmarks(particle):
    """Helper function to add some landmarks and their covariances."""
    # Add a landmark, at (500,-500), with standard deviation 100.
    particle.landmark_positions.append(np.array([500.0, -500.0]))
    particle.landmark_covariances.append(np.array(
        [[ 100.0**2,   0.0 ],
         [   0.0, 100.0**2 ]]))
    # Add two landmarks along the x axis, at 1000 and 2000, with
    # different covariances.
    particle.landmark_positions.append(np.array([1000.0, 0.0]))
    particle.landmark_covariances.append(np.array(
        [[ 100.0**2,   0.0 ],
         [   0.0, 100.0**2 ]]))
    particle.landmark_positions.append(np.array([2000.0, 0.0]))
    particle.landmark_covariances.append(np.array(
        [[ 200.0**2, -100.0**2 ],
         [-100.0**2,  200.0**2 ]]))

if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0

    # Filter constants.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.
    # What I called Qz
    Qt_measurement_covariance = \
        np.diag([measurement_distance_stddev**2,
                 measurement_angle_stddev**2])

    # Define a particle: position (x, y) and orientation.
    p = Particle(np.array([-scanner_displacement, 0.0, 0.0]))
    add_landmarks(p)
    N = p.number_of_landmarks()

    # Compute expected measurements to all the landmarks within the list
    # of landmarks for the current particle.
    for i in xrange(N):
        print "Landmark", i, "----------"
        em = p.h_expected_measurement_for_landmark(i, scanner_displacement)
        print " Expected range:", em[0], "bearing [deg]", em[1]/pi*180
        H, Ql = p.H_Ql_jacobian_and_measurement_covariance_for_landmark(
            i, Qt_measurement_covariance, scanner_displacement)
        print " Covariance of measurement:\n", Ql
        print " H matrix:\n", H

    # Compute correspondence likelihoods.
    # Define a set of measurements: (range, bearing).
    print "Measurement likelihoods ----------"
    measurements = [
        ("close to landmark 0", np.array([500*sqrt(2), -pi/4])),
        ("exactly between landmark 1 and 2", np.array([1500.0, 0.0]))
        ]
    for (text, m) in measurements:
        print "Likelihoods for measurement", text
        likelihoods = p.compute_correspondence_likelihoods(
            m, N, Qt_measurement_covariance, scanner_displacement)
        print likelihoods
