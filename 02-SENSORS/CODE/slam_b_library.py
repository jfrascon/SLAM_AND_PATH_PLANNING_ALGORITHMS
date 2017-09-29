# This file contains some of the functions developed in the
# SLAM_A unit.
# slam_b_library
# Claus Brenner, 17.11.2012
from math import sin, cos, pi
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):

    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        dist = motor_ticks[0] * ticks_to_mm
        theta = old_pose[2]
        x = old_pose[0] + dist * cos(theta)
        y = old_pose[1] + dist * sin(theta)
        return (x, y, theta)

    else:
        # Turn. Compute alpha, R, etc.
        # Get old center
        old_theta = old_pose[2]
        old_x = old_pose[0]
        old_y = old_pose[1]

        # Modification: subtract offset to compute center.
        old_x -= cos(old_theta) * scanner_displacement
        old_y -= sin(old_theta) * scanner_displacement

        l = motor_ticks[0] * ticks_to_mm
        r = motor_ticks[1] * ticks_to_mm
        alpha = (r - l) / robot_width
        R = l / alpha
        new_theta = (old_theta + alpha) % (2*pi)
        new_x = old_x + (R + robot_width/2.0) * (sin(new_theta) - sin(old_theta))
        new_y = old_y + (R + robot_width/2.0) * (-cos(new_theta) + cos(old_theta))

        # Modification: add offset to compute location of scanner.
        new_x += cos(new_theta) * scanner_displacement
        new_y += sin(new_theta) * scanner_displacement
        
        return (new_x, new_y, new_theta)

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

# Given detected cylinder coordinates: (beam_id, distance), return
# cartesian coordinates (x, y). This is a polar to cartesian conversion
# with an added offset.
def compute_cartesian_coordinates(cylinders, cylinder_offset):
    result = []
    for c in cylinders:
        angle = LegoLogfile.beam_index_to_angle(c[0])
        r = c[1] + cylinder_offset
        result.append( (r*cos(angle), r*sin(angle)) )
    return result

# Returns a new similarity transform, which is the concatenation of
# transform a and b, "a after b".
# The transform is described in the form of:
# (scale, cos(angle), sin(angle), translate_x, translate_y)
# i.e., the angle is described by a direction vector.
def concatenate_transform(a, b):
    laa, ca, sa, txa, tya = a
    lab, cb, sb, txb, tyb = b

    # New lambda is multiplication.
    la = laa * lab

    # New rotation matrix uses trigonometric angle sum theorems.
    c = ca*cb - sa*sb
    s = sa*cb + ca*sb

    # New translation is a translation plus rotated b translation.
    tx = txa + laa * ca * txb - laa * sa * tyb
    ty = tya + laa * sa * txb + laa * ca * tyb

    return (la, c, s, tx, ty)
