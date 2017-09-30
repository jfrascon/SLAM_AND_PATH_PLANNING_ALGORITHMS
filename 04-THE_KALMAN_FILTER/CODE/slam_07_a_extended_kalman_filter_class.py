# This introduces the extended Kalman Filter class.
# The computational steps are the same as in Unit A,
# slam_02_b_filter_motor_file.
#
# slam_07_a_extended_kalman_filter_class
# Claus Brenner, 06.12.2012
from lego_robot import *
from math import sin, cos, pi
from numpy import *

class ExtendedKalmanFilter:

    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi # Express g3 between [-pi and pi]
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return array([g1, g2, g3])


if __name__ == '__main__':
    # Constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Measured start position.
    # This is the origin point for the laser. See SLAM-A-Getting_started/11-Getting_started-Files/slam_02_b_filter_motor_file_question.py
    state = array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    # We have to obtain the origin point for the center of the robot.
    state = array([state[0] - scanner_displacement*cos(state[2]), state[1] - scanner_displacement*sin(state[2]), state[2]])
    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records and generate a filtered position list.
    states = []
    for ticks in logfile.motor_ticks:
        # Prediction (so far only predicts state, not covariance).
        control = array(ticks) * ticks_to_mm
        state = ExtendedKalmanFilter.g(state, control, robot_width)

        # Log state.
        states.append(state)

    # Write all states to file.
    with open("states_from_ticks.txt", "w") as fh:
        for s in states:
            # Output the center of the scanner, not the center of the robot.
            fh.write("F {} {} {}\n".format(s[0] + scanner_displacement * cos(s[2]),  s[1] + scanner_displacement * sin(s[2]), s[2] + 0.0))
