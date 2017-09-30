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
# with the corresponding cylinder in the reference dataset.
# In detail:
# - It takes scan data and detects cylinders.
# - For every cylinder, it computes its world coordinate using
#   the polar coordinates from the cylinder detection and the robot's pose,
#   taking into account the scanner's displacement.
# - Using the world coordinate, it finds the closest cylinder in the
#   reference_cylinder list, which has a distance of at most
#   max_reference_distance.
# - If there is such a closest cylinder, the (distance, angle) pair from the
#   scan measurement (these are the two original observations) and the matched
#   cylinder from reference_cylinders are added to the output list.
# - This is repeated for every cylinder detected in the scan.
def get_observations(scan, jump, min_dist, cylinder_offset,
                     robot_pose, scanner_displacement,
                     reference_cylinders, max_reference_distance):
    der = compute_derivative(scan, min_dist)
    cylinders = find_cylinders(scan, der, jump, min_dist)
    # Compute scanner pose from robot pose.
    scanner_pose = (robot_pose[0] + cos(robot_pose[2]) * scanner_displacement,
                    robot_pose[1] + sin(robot_pose[2]) * scanner_displacement,
                    robot_pose[2])

    # For every detected cylinder which has a closest matching pole in the
    # reference cylinders set, put the measurement (distance, angle) and the
    # corresponding reference cylinder into the result list.
    result = []
    for c in cylinders:
        # Compute the angle and distance measurements.
        angle = LegoLogfile.beam_index_to_angle(c[0])
        distance = c[1] + cylinder_offset
        # Compute x, y of cylinder in world coordinates.
        x, y = distance*cos(angle), distance*sin(angle)
        x, y = LegoLogfile.scanner_to_world(scanner_pose, (x, y))
        # Find closest cylinder in reference cylinder set.
        best_dist_2 = max_reference_distance * max_reference_distance
        best_ref = None
        for ref in reference_cylinders:
            dx, dy = ref[0] - x, ref[1] - y
            dist_2 = dx * dx + dy * dy
            if dist_2 < best_dist_2:
                best_dist_2 = dist_2
                best_ref = ref
        # If found, add to both lists.
        if best_ref:
            result.append(((distance, angle), best_ref))

    return result
