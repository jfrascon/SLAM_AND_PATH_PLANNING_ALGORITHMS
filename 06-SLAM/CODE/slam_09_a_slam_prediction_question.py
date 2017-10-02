# EKF SLAM - prediction step.
#
# slam_09_a_slam_prediction
# Claus Brenner, 20 JAN 13
from lego_robot import *
from math import sin, cos, pi, atan2, sqrt
from numpy import *

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
        G3 = self.dg_dstate(self.state, control, self.robot_width)
        left, right = control
        left_var = (self.control_motion_factor * left)**2 +\
                   (self.control_turn_factor * (left-right))**2
        right_var = (self.control_motion_factor * right)**2 +\
                    (self.control_turn_factor * (left-right))**2
        control_covariance = diag([left_var, right_var])
        V = self.dg_dcontrol(self.state, control, self.robot_width)
        R3 = dot(V, dot(control_covariance, V.T))

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

        # Now enlarge G3 and R3 to accomodate all landmarks. Then, compute the
        # new covariance matrix self.covariance.
        GN = zeros((3 + 2*self.number_of_landmarks, 3 + 2*self.number_of_landmarks))
        GN[0:3, 0:3] = G3
        GN[3:, 3:] = eye(2*self.number_of_landmarks, 2*self.number_of_landmarks)

        RN = zeros((3 + 2*self.number_of_landmarks, 3 + 2*self.number_of_landmarks))
        RN[0:3, 0:3] = R3

        self.covariance = dot(GN, dot(self.covariance, GN.T)) + RN

        # state' = g(state, control)
        self.state = self.g(self.state[0:3], control, self.robot_width)

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
    # I set the initial covariance to zero because I'm very
    # about my initial position and orientation, simply, because
    # I defined my map to have its origin and its orientation
    # according to my initial robot's pose.
    initial_covariance = zeros((3,3))

    # Setup filter.
    kf = ExtendedKalmanFilterSLAM(initial_state, initial_covariance,
                                  robot_width, scanner_displacement,
                                  control_motion_factor, control_turn_factor)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records and all measurements and generate
    # filtered positions and covariances.
    # This is the EKF SLAM loop.
    f = open("ekf_slam_prediction.txt", "w")
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

    f.close()
