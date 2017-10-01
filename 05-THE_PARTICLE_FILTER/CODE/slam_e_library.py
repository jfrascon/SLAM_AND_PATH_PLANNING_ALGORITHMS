# This file contains helper functions for Unit E of the SLAM lecture.
# Claus Brenner, 05 JAN 2013
from math import sin, cos, pi
from lego_robot import LegoLogfile

# Find the derivative in scan data, ignoring invalid measurements.


def compute_derivative(scan, min_dist):
    jumps = [0]
    for i in xrange(1, len(scan) - 1):
        l = scan[i - 1]
        r = scan[i + 1]
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
                cylinder_list.append((sum_ray / rays, sum_depth / rays))
            on_cylinder = False
        # Always add point, if it is a valid measurement.
        elif scan[i] > min_dist:
            sum_ray += i
            sum_depth += scan[i]
            rays += 1
    return cylinder_list

# Detects cylinders and computes bearing, distance and cartesian coordinates (in
# the scanner's coordinate system).
# Result is a list of tuples: (range, bearing, x, y).


def get_cylinders_from_scan(scan, jump, min_dist, cylinder_offset):
    der = compute_derivative(scan, min_dist)
    cylinders = find_cylinders(scan, der, jump, min_dist)
    result = []
    for c in cylinders:
        # Compute the angle and distance measurements.
        bearing = LegoLogfile.beam_index_to_angle(c[0])
        distance = c[1] + cylinder_offset
        # Compute x, y of cylinder in the scanner system.
        x, y = distance * cos(bearing), distance * sin(bearing)
        result.append((distance, bearing, x, y))
    return result

# For a given pose, assign cylinders.
# cylinders is a list of cylinder measurements
#  (range, bearing, x, y)
#  where x, y are cartesian coordinates in the scanner's system.
# Returns a list of matches, where each element is a tuple of 2 tuples:
#  [ ((range_0, bearing_0), (landmark_x, landmark_y)), ... ]


def assign_cylinders(cylinders, robot_pose, scanner_displacement,
                     reference_cylinders):
    # Compute scanner pose from robot pose.
    scanner_pose = (robot_pose[0] + cos(robot_pose[2]) * scanner_displacement,
                    robot_pose[1] + sin(robot_pose[2]) * scanner_displacement,
                    robot_pose[2])

    # Find closest cylinders.
    result = []
    for c in cylinders:
        # Get world coordinate of cylinder.
        x, y = LegoLogfile.scanner_to_world(scanner_pose, c[2:4])
        # Find closest cylinder in reference cylinder set.
        best_dist_2 = 1e300
        best_ref = None
        for ref in reference_cylinders:
            dx, dy = ref[0] - x, ref[1] - y
            dist_2 = dx * dx + dy * dy
            if dist_2 < best_dist_2:
                best_dist_2 = dist_2
                best_ref = ref
        # If found, add to both lists.
        if best_ref:
            result.append((c[0:2], best_ref))

    return result
