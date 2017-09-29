# For each cylinder in the scan, find its cartesian coordinates,
# in the world coordinate system.
# Find the closest pairs of cylinders from the scanner and cylinders
# from the reference, and the optimal transformation which aligns them.
# Then, use this transform to correct the pose.
# 04_d_apply_transform
# Claus Brenner, 14 NOV 2012
from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import\
     compute_scanner_cylinders, write_cylinders
from math import sqrt, atan2
import sys

# Given a list of cylinders (points) and reference_cylinders:
# For every cylinder, find the closest reference_cylinder and add
# the index pair (i, j), where i is the index of the cylinder, and
# j is the index of the reference_cylinder, to the result list.
# This is the function developed in slam_04_b_find_cylinder_pairs.
def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []

    # --->>> Enter your code here.
    # Make a loop over all cylinders and reference_cylinders.
    # In the loop, if cylinders[i] is closest to reference_cylinders[j],
    # and their distance is below max_radius, then add the
    # tuple (i,j) to cylinder_pairs, i.e., cylinder_pairs.append( (i,j) ).

    max_radius_squared = max_radius**2
    index_j = len(reference_cylinders)
    #print("max_radius_squared: {}".format(max_radius_squared))

    for i in xrange(0, len(cylinders)):
        min_dist = sys.float_info.max
        for j in xrange(0, len(reference_cylinders)):
            # No need to use sqrt, it's the same to compare sqrt values as squared values when it
            # comes to obtain which is lesser.
            dist = (cylinders[i][0] - reference_cylinders[j][0])**2 + (cylinders[i][1] - reference_cylinders[j][1])**2
            #print("{} {} {}".format(i, j, dist))
            if(dist < min_dist):
                min_dist = dist
                index_j = j
                #print("min_dist: {} - j: {}".format(min_dist, index_j))

        if(min_dist < max_radius_squared):
            #print("*****{} {} {}".format(min_dist, i, index_j))
            cylinder_pairs.append((i, index_j))

    return cylinder_pairs

# Given a point list, return the center of mass.
def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (sx / len(point_list), sy / len(point_list))

# Given a left_list of points and a right_list of points, compute
# the parameters of a similarity transform: scale, rotation, translation.
# If fix_scale is True, use the fixed scale of 1.0.
# The returned value is a tuple of:
# (scale, cos(angle), sin(angle), x_translation, y_translation)
# i.e., the rotation angle is not given in radians, but rather in terms
# of the cosine and sine.
def estimate_transform(left_list, right_list, fix_scale = False):
    # Compute left and right center.
    lc = compute_center(left_list)
    rc = compute_center(right_list)

    cs = 0
    ss = 0
    rr = 0
    ll = 0

    #print("lc_x: {} lc_y: {}".format(lc[0], lc[1]))
    #print("rc_x: {} rc_y: {}".format(rc[0], rc[1]))

    # --->>> Insert here your code to compute lambda, c, s and tx, ty.
    for i in xrange(0, len(left_list)):
        lix_p = left_list[i][0] - lc[0]
        liy_p = left_list[i][1] - lc[1]

        #print("llx: {} lly: {}".format(left_list[i][0], left_list[i][1]))
        #print("lix_p: {} liy_p: {}".format(lix_p, liy_p))

        rix_p = right_list[i][0] - rc[0]
        riy_p = right_list[i][1] - rc[1]

        #print("rrx: {} rry: {}".format(right_list[i][0], right_list[i][1]))
        #print("rix_p: {} riy_p: {}".format(rix_p, riy_p))

        cs += rix_p*lix_p + riy_p*liy_p
        ss += riy_p*lix_p - rix_p*liy_p

        rr += rix_p*rix_p + riy_p*riy_p
        ll += lix_p*lix_p + liy_p*liy_p

    # We have to estimate 4 parameters: cs, ss, rr, ll.
    # So, we need at least 4 points, 2 points in the left list and 2 points in the
    # right list. It means 4 x-coordinates and 4 y-coordinates.

    # If we only have 2 points, this is, one in each list, we'll have problems when it comes
    # to compute the terms 'la', 'c' and 's' because cs = ss = rr = ll = 0. So, we'll be able to
    # compute only 'tx' and 'ty'.
    # With only one point in left_list and right_list:
    #   lc_x = left_list_x  --> lix_p = 0 (the same for y coordinate)
    #   rc_x = right_list_x --> rix_p = 0 (the same for y coordinate)
    #   cs = ss = rr = ll = 0 --> coeff = 0
    # --> c=0/INF!!!!!!
    # --> c=0/INF!!!!!!
    # The same problem will arise if we have multiple points but all of them have the same
    # x and y coordinates.
    # In this 2 situations we have to return None.
    if (not rr and not ll):
        return None

    la = 1 if(fix_scale) else sqrt(rr/ll)
    coeff = sqrt(cs**2 + ss**2)
    #print("cs: {} ss: {} rr: {} ll: {}".format(cs, ss, rr, ll))
    #print("coeff: {}".format(coeff))
    c = cs/coeff
    s = ss/coeff
    tx = rc[0] - la*c*lc[0] + la*s*lc[1]
    ty = rc[1] - la*s*lc[0] - la*c*lc[1]

    #print("la: {} c: {} s: {} tx: {} ty: {}".format(la, c, s, tx, ty))
    #print("\n")

    return la, c, s, tx, ty

# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)

# Correct the pose = (x, y, heading) of the robot using the given
# similarity transform. Note this changes the position as well as
# the heading.
def correct_pose(pose, trafo):
    # --->>> This is what you'll have to implement.
    x, y = apply_transform(trafo, pose[:2])
    # atan2 calculates angles between [-180, 180]
    # There's no need to apply the modulus operator,
    # because it's the same 230 (150 + 80) as -130 (20 - 150)
    # in terms of cosine and sine.
    theta = pose[2] + atan2(trafo[2], trafo[1])
    return (x, y, theta)  # Replace this by the corrected pose.


if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The constants we used for the cylinder detection in our scan.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # The maximum distance allowed for cylinder assignment.
    max_cylinder_distance = 400.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Also read the reference cylinders (this is our map).
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    out_file = file("apply_transform.txt", "w")
    for i in xrange(len(logfile.scan_data)):
        # Compute the new pose.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Extract cylinders, also convert them to world coordinates.
        cartesian_cylinders = compute_scanner_cylinders(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset)
        world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                           for c in cartesian_cylinders]

        # For every cylinder, find the closest reference cylinder.
        cylinder_pairs = find_cylinder_pairs(
            world_cylinders, reference_cylinders, max_cylinder_distance)

        # Estimate a transformation using the cylinder pairs.
        trafo = estimate_transform(
            [world_cylinders[pair[0]] for pair in cylinder_pairs],
            [reference_cylinders[pair[1]] for pair in cylinder_pairs],
            fix_scale = True)

        # Transform the cylinders using the estimated transform.
        transformed_world_cylinders = []
        if trafo:
            transformed_world_cylinders =\
                [apply_transform(trafo, c) for c in
                 [world_cylinders[pair[0]] for pair in cylinder_pairs]]

        # Also apply the trafo to correct the position and heading.
        if trafo:
            pose = correct_pose(pose, trafo)

        # Write to file.
        # The pose.
        out_file.write("F %f %f %f\n" % pose)
        # The detected cylinders in the scanner's coordinate system.
        write_cylinders(out_file, "D C", cartesian_cylinders)
        # The detected cylinders, transformed using the estimated trafo.
        write_cylinders(out_file, "W C", transformed_world_cylinders)

    out_file.close()
