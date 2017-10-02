# EKF SLAM - adding one landmark.
#
# slam_09_b_slam_add_landmark
# Claus Brenner, 20 JAN 13
from lego_robot import *
from math import sin, cos, pi, atan2, sqrt
from numpy import *
from slam_f_library import write_cylinders, write_error_ellipses


class ExtendedKalmanFilterSLAM:
    def __init__(self, state, covariance,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor

        # Currently, the number of landmarks is zero.
        self.number_of_landmarks = 0

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

        return array([g1, g2, g3])

    @staticmethod
    def dg_dstate(state, control, w):
        theta = state[2]
        l, r = control
        if r != l:
            alpha = (r-l)/w
            theta_ = theta + alpha
            rpw2 = l/alpha + w/2.0
            m = array([[1.0, 0.0, rpw2*(cos(theta_) - cos(theta))],
                       [0.0, 1.0, rpw2*(sin(theta_) - sin(theta))],
                       [0.0, 0.0, 1.0]])
        else:
            m = array([[1.0, 0.0, -l*sin(theta)],
                       [0.0, 1.0,  l*cos(theta)],
                       [0.0, 0.0,  1.0]])
        return m

    @staticmethod
    def dg_dcontrol(state, control, w):
        theta = state[2]
        l, r = tuple(control)
        if r != l:
            rml = r - l
            rml2 = rml * rml
            theta_ = theta + rml/w
            dg1dl = w*r/rml2*(sin(theta_)-sin(theta))  - (r+l)/(2*rml)*cos(theta_)
            dg2dl = w*r/rml2*(-cos(theta_)+cos(theta)) - (r+l)/(2*rml)*sin(theta_)
            dg1dr = (-w*l)/rml2*(sin(theta_)-sin(theta)) + (r+l)/(2*rml)*cos(theta_)
            dg2dr = (-w*l)/rml2*(-cos(theta_)+cos(theta)) + (r+l)/(2*rml)*sin(theta_)

        else:
            dg1dl = 0.5*(cos(theta) + l/w*sin(theta))
            dg2dl = 0.5*(sin(theta) - l/w*cos(theta))
            dg1dr = 0.5*(-l/w*sin(theta) + cos(theta))
            dg2dr = 0.5*(l/w*cos(theta) + sin(theta))

        dg3dl = -1.0/w
        dg3dr = 1.0/w
        m = array([[dg1dl, dg1dr], [dg2dl, dg2dr], [dg3dl, dg3dr]])

        return m

    def predict(self, control):
        """The prediction step of the Kalman filter."""
        # covariance' = G * covariance * GT + R
        # where R = V * (covariance in control space) * VT.
        # Covariance in control space depends on move distance.

        # --->>> Put your code here.

        # Hints:
        # - The number of landmarks is self.number_of_landmarks.
        # - eye(n) is the numpy function which returns a n x n identity matrix.
        # - zeros((n,n)) returns a n x n matrix which is all zero.
        # - If M is a matrix, M[0:2,1:5] returns the submatrix which consists
        #   of the rows 0 and 1 (but not 2) and the columns 1, 2, 3, 4.
        #   This submatrix operator can be used on either side of an assignment.
        # - Similarly for vectors: v[1:3] returns the vector consisting of the
        #   elements 1 and 2, but not 3.
        # - All matrix and vector indices start at 0.

        G3 = self.dg_dstate(self.state, control, self.robot_width)

        left, right = control
        left_var = (self.control_motion_factor * left)**2 +\
                   (self.control_turn_factor * (left-right))**2
        right_var = (self.control_motion_factor * right)**2 +\
                    (self.control_turn_factor * (left-right))**2
        control_covariance = diag([left_var, right_var])
        V = self.dg_dcontrol(self.state, control, self.robot_width)
        R3 = dot(V, dot(control_covariance, V.T))

        # Now enlarge G3 and R3 to accomodate all landmarks. Then, compute the
        # new covariance matrix self.covariance.
        new_offset = 2*self.number_of_landmarks
        G = zeros((3 + new_offset, 3 + new_offset))
        G[0:3, 0:3] = G3
        G[3:, 3:] = eye(new_offset, new_offset)

        R = zeros((3 + new_offset, 3 + new_offset))
        R[0:3, 0:3] = R3

        self.covariance = dot(G, dot(self.covariance, G.T)) + R

        # state' = g(state, control)
        self.state[0:3] = self.g(self.state[0:3], control, self.robot_width)

    def add_landmark_to_state(self, initial_coords):
        """Enlarge the current state and covariance matrix to include one more
           landmark, which is given by its initial_coords (an (x, y) tuple).
           Returns the index of the newly added landmark."""

        # --->>> Put here your new code to augment the robot's state and
        #        covariance matrix.
        #        Initialize the state with the given initial_coords and the
        #        covariance with 1e10 (as an approximation for "infinity".
        # Hints:
        # - If M is a matrix, use M[i:j,k:l] to obtain the submatrix of
        #   rows i to j-1 and colums k to l-1. This can be used on the left and
        #   right side of the assignment operator.
        # - zeros(n) gives a zero vector of length n, eye(n) an n x n identity
        #   matrix.
        # - Do not forget to increment self.number_of_landmarks.
        # - Do not forget to return the index of the newly added landmark. I.e.,
        #   the first call should return 0, the second should return 1.
        landmark_index = self.number_of_landmarks
        old_offset = 2*self.number_of_landmarks
        self.number_of_landmarks += 1
        new_offset = 2*self.number_of_landmarks

        covarianceN = zeros((3 + new_offset, 3 + new_offset))
        covarianceN[0:3+old_offset, 0:3+old_offset] = self.covariance
        covarianceN[-2,-2] = covarianceN[-1,-1] = 1e10
        self.covariance = covarianceN

        state_prime = zeros(3 + new_offset)
        state_prime[0:3+old_offset] = self.state
        state_prime[-2] = initial_coords[0]
        state_prime[-1] = initial_coords[1]
        self.state = state_prime

        return landmark_index

    def get_landmarks(self):
        """Returns a list of (x, y) tuples of all landmark positions."""
        return ([(self.state[3+2*j], self.state[3+2*j+1])
                 for j in xrange(self.number_of_landmarks)])

    def get_landmark_error_ellipses(self):
        """Returns a list of all error ellipses, one for each landmark."""
        ellipses = []
        for i in xrange(self.number_of_landmarks):
            j = 3 + 2 * i
            ellipses.append(self.get_error_ellipse(
                self.covariance[j:j+2, j:j+2]))
        return ellipses

    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.

    # Arbitrary start position.
    initial_state = array([500.0, 0.0, 45.0 / 180.0 * pi])

    # Covariance at start position.
    initial_covariance = zeros((3,3))

    # Setup filter.
    kf = ExtendedKalmanFilterSLAM(initial_state, initial_covariance,
                                  robot_width, scanner_displacement,
                                  control_motion_factor, control_turn_factor)

    # Just to test the algorithm, add one landmark.
    kf.add_landmark_to_state((400.0, 700.0))

    # To make the error ellipse visible, set a smaller variance.
    if kf.number_of_landmarks > 0:
        kf.covariance[-2,-2] = 300.0**2  # 300 mm in x.
        kf.covariance[-1,-1] = 500.0**2  # 500 mm in y.

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records and all measurements and generate
    # filtered positions and covariances.
    # This is the EKF SLAM loop.
    f = open("ekf_slam_add_landmarks.txt", "w")
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        control = array(logfile.motor_ticks[i]) * ticks_to_mm
        kf.predict(control)

        # End of EKF SLAM - from here on, data is written.

        # Output the center of the scanner, not the center of the robot.
        print >> f, "F %f %f %f" % \
            tuple(kf.state[0:3] + [scanner_displacement * cos(kf.state[2]),
                                   scanner_displacement * sin(kf.state[2]),
                                   0.0])
        # Write covariance matrix in angle stddev1 stddev2 stddev-heading form
        e = ExtendedKalmanFilterSLAM.get_error_ellipse(kf.covariance)
        print >> f, "E %f %f %f %f" % (e + (sqrt(kf.covariance[2,2]),))
        # Write estimates of landmarks.
        write_cylinders(f, "W C", kf.get_landmarks())
        # Write error ellipses of landmarks.
        write_error_ellipses(f, "W E", kf.get_landmark_error_ellipses())

    f.close()
