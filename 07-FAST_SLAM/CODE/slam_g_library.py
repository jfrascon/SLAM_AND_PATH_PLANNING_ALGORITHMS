# This file contains helper functions for Unit G of the SLAM lecture.
# Claus Brenner, 27 JAN 2013
from math import sin, cos, atan2, sqrt, pi
from lego_robot import LegoLogfile
import numpy as np

# Utility to write a list of cylinders to (one line of) a given file.
# Line header defines the start of each line, e.g. "D C" for a detected
# cylinder or "W C" for a world cylinder.
def write_cylinders(file_desc, line_header, cylinder_list):
    print >> file_desc, line_header,
    for c in cylinder_list:
        print >> file_desc, "%.1f %.1f" % tuple(c),
    print >> file_desc

# Utility to write a list of error ellipses to (one line of) a given file.
# Line header defines the start of each line.
# Note that in contrast to previous versions, this takes a list of covariance
# matrices instead of list of ellipses.
def write_error_ellipses(file_desc, line_header, covariance_matrix_list):
    print >> file_desc, line_header,
    for m in covariance_matrix_list:
        eigenvals, eigenvects = np.linalg.eig(m)
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        print >> file_desc, "%.3f %.1f %.1f" % \
                 (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1])),
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

# Detects cylinders and computes range, bearing and cartesian coordinates
# (in the scanner's coordinate system).
# The result is modified from previous versions: it returns a list of
# tuples of two numpy arrays, the first being (distance, bearing), the second
# being (x, y) in the scanner's coordinate system.
def get_cylinders_from_scan(scan, jump, min_dist, cylinder_offset):
    der = compute_derivative(scan, min_dist)
    cylinders = find_cylinders(scan, der, jump, min_dist)
    result = []
    for c in cylinders:
        # Compute the angle and distance measurements.
        bearing = LegoLogfile.beam_index_to_angle(c[0])
        distance = c[1] + cylinder_offset
        # Compute x, y of cylinder in the scanner system.
        x, y = distance*cos(bearing), distance*sin(bearing)
        result.append( (np.array([distance, bearing]), np.array([x, y])) )
    return result

def get_mean(particles):
    """Compute mean position and heading from a given set of particles."""
    # Note this function would more likely be a part of FastSLAM or a base class
    # of FastSLAM. It has been moved here for the purpose of keeping the
    # FastSLAM class short in this tutorial.
    mean_x, mean_y = 0.0, 0.0
    head_x, head_y = 0.0, 0.0
    for p in particles:
        x, y, theta = p.pose
        mean_x += x
        mean_y += y
        # The way the mean of the heading is calculated was presented at lecture on the
        # Particle Filter (lecture E).
        head_x += cos(theta)
        head_y += sin(theta)
    n = max(1, len(particles))
    # All the heading (each heading for each particle) is in the range [-pi, pi]. This fact is
    # controlled in the g method located in the class Particle.
    # mean_sin = sum{sin(...)}/N
    # mean_cos = sum{cos(...)}/N
    # mean_tan = mean_sin/mean_cos = sum{sin(...)}/N / sum{cos(...)}/N = sum{sin(...)} / sum{cos(...)}
    # mean_heading = atan2(mean_tan) = atan2(sum{sin(...)} / sum{cos(...)})
    # Therefore, if all the headings are in this range, [-pi, pi], the mean heading is also in this
    # range, so there's no need to use the tecnique (mean_heading + pi) % (2*pi) - pi
    return np.array([mean_x / n, mean_y / n, atan2(head_y, head_x)])

def get_error_ellipse_and_heading_variance(particles, mean):
    """Given a set of particles and their mean (computed by get_mean()),
       returns a tuple: (angle, stddev1, stddev2, heading-stddev) which is
       the orientation of the xy error ellipse, the half axis 1, half axis 2,
       and the standard deviation of the heading."""
    # Note this function would more likely be a part of FastSLAM or a base class
    # of FastSLAM. It has been moved here for the purpose of keeping the
    # FastSLAM class short in this tutorial.
    center_x, center_y, center_heading = mean
    n = len(particles)
    if n < 2:
        return (0.0, 0.0, 0.0, 0.0)

    # Compute covariance matrix in xy.
    sxx, sxy, syy = 0.0, 0.0, 0.0
    for p in particles:
        x, y, theta = p.pose
        dx = x - center_x
        dy = y - center_y
        sxx += dx * dx
        sxy += dx * dy
        syy += dy * dy
    cov_xy = np.array([[sxx, sxy], [sxy, syy]]) / (n-1)

    # Get variance of heading.
    var_heading = 0.0
    for p in particles:
        dh = (p.pose[2] - center_heading + pi) % (2*pi) - pi
        var_heading += dh * dh
    var_heading = var_heading / (n-1)

    # Convert xy to error ellipse.
    eigenvals, eigenvects = np.linalg.eig(cov_xy)
    ellipse_angle = atan2(eigenvects[1,0], eigenvects[0,0])

    return (ellipse_angle, sqrt(abs(eigenvals[0])),
            sqrt(abs(eigenvals[1])),
            sqrt(var_heading))

def print_particles(particles, file_desc):
    # Note this function would more likely be a part of FastSLAM or a base class
    # of FastSLAM. It has been moved here for the purpose of keeping the
    # FastSLAM class short in this tutorial.
    """Prints particles to given file_desc output."""
    if not particles:
        return
    print >> file_desc, "PA",
    for p in particles:
        print >> file_desc, "%.0f %.0f %.3f" % tuple(p.pose),
    print >> file_desc
