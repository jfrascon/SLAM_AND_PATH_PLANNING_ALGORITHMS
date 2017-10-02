# This file contains helper functions for Unit D of the SLAM lecture,
# most of which were developed in earlier units.
# Claus Brenner, 11 DEC 2012
from math import sin, cos, pi
from lego_robot import LegoLogfile

# Utility to write a list of cylinders to (one line of) a given file.
# Line header defines the start of each line, e.g. "D C" for a detected
# cylinder or "W C" for a world cylinder.
def write_cylinders(file_desc, line_header, cylinder_list):
    print >> file_desc, line_header,
    for c in cylinder_list:
        print >> file_desc, "%.1f %.1f" % c,
    print >> file_desc

# Utility to write a list of error ellipses to (one line of) a given file.
# Line header defines the start of each line.
def write_error_ellipses(file_desc, line_header, error_ellipse_list):
    print >> file_desc, line_header,
    for e in error_ellipse_list:
        print >> file_desc, "%.3f %.1f %.1f" % e,
    print >> file_desc

# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):
    jumps = [ 0 ]
    for i in xrange(1, len(scan) - 1):
        l = scan[i-1]
        r = scan[i+1]
        if l > min_dist and r > min_dist:
            derivative = (r - l) / 2.0
            jumps.append(derivative)
        else:
            jumps.append(0)
    jumps.append(0)
    return jumps

# For each area between a left falling edge and a right rising edge,
# determine the average ray number and the average depth.
def find_cylinders(scan, scan_derivative, jump, min_dist):
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0

    for i in xrange(len(scan_derivative)):
        if scan_derivative[i] < -jump:
            # Start a new cylinder, independent of on_cylinder.
            on_cylinder = True
            sum_ray, sum_depth, rays = 0.0, 0.0, 0
        elif scan_derivative[i] > jump:
            # Save cylinder if there was one.
            if on_cylinder and rays:
                cylinder_list.append((sum_ray/rays, sum_depth/rays))
            on_cylinder = False
        # Always add point, if it is a valid measurement.
        elif scan[i] > min_dist:
            sum_ray += i
            sum_depth += scan[i]
            rays += 1
    return cylinder_list

# This function does all processing needed to obtain the cylinder observations.
# It matches the cylinders and returns distance and angle observations together
# with the cylinder coordinates in the world system, the scanner
# system, and the corresponding cylinder index (in the list of estimated parameters).
# In detail:
# - It takes scan data and detects cylinders.
# - For every detected cylinder, it computes its world coordinate using
#   the polar coordinates from the cylinder detection and the robot's pose,
#   taking into account the scanner's displacement.
# - Using the world coordinate, it finds the closest cylinder in the
#   list of current (estimated) landmarks, which are part of the current state.
#
# - If there is such a closest cylinder, the (distance, angle) pair from the
#   scan measurement (these are the two observations), the (x, y) world
#   coordinates of the cylinder as determined by the measurement, the (x, y)
#   coordinates of the same cylinder in the scanner's coordinate system,
#   and the index of the matched cylinder are added to the output list.
#   The index is the cylinder number in the robot's current state.
# - If there is no matching cylinder, the returned index will be -1.
def get_observations(scan, jump, min_dist, cylinder_offset,
                     robot,
                     max_cylinder_distance):
    der = compute_derivative(scan, min_dist)
    cylinders = find_cylinders(scan, der, jump, min_dist)
    # Compute scanner pose from robot pose.
    scanner_pose = (
        robot.specific_state[0] + cos(robot.specific_state[2]) * robot.scanner_displacement,
        robot.specific_state[1] + sin(robot.specific_state[2]) * robot.scanner_displacement,
        robot.specific_state[2])

    # For every detected cylinder which has a closest matching pole in the
    # cylinders that are part of the current state, put the measurement
    # (distance, angle) and the corresponding cylinder index into the result list.
    result = []
    for c in cylinders:
        # Compute the angle and distance measurements.
        angle = LegoLogfile.beam_index_to_angle(c[0])
        distance = c[1] + cylinder_offset
        # Compute x, y of cylinder in world coordinates.
        xs, ys = distance*cos(angle), distance*sin(angle)
        x, y = LegoLogfile.scanner_to_world(scanner_pose, (xs, ys))
        # Find closest cylinder in the state.
        best_dist_2 = max_cylinder_distance * max_cylinder_distance
        best_index = -1
        for index in xrange(robot.number_of_landmarks):
            pole_x, pole_y = robot.specific_state[3+2*index : 3+2*index+2]
            dx, dy = pole_x - x, pole_y - y
            dist_2 = dx * dx + dy * dy
            if dist_2 < best_dist_2:
                best_dist_2 = dist_2
                best_index = index
        # Always add result to list. Note best_index may be -1.
        result.append(((distance, angle), (x, y), (xs, ys), best_index))

    return result
