# For each cylinder in the scan, find its cartesian coordinates,
# in the scanner's coordinate system.
# Write the result to a file which contains all cylinders, for all scans.
# 03_d_find_cylinders_cartesian
# Claus Brenner, 09 NOV 2012
from lego_robot import *
from math import sin, cos

# Find the derivative of at least min_jump in scan data,
# ignoring invalid measurements.
# return -1 for a decreasing derivative, +1 for an increasing derivative.
def compute_derivative(scan, min_dist):
    jumps = [ 0 ]
    for i in xrange(1, len(scan) - 1):
        # --->>> Insert your code here.
        # Compute derivative using formula "(f(i+1) - f(i-1)) / 2".
        # Do not use erroneous scan values, which are below min_dist.
        l = scan[i-1]
        r = scan[i+1]
        if(l > min_dist and r > min_dist):
            der = (r - l)/2.0
            jumps.append(der)
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
        # --->>> Insert your cylinder code here.
        # Whenever you find a cylinder, add a tuple
        # (average_ray, average_depth) to the cylinder_list.
        # If I find a strong negative value for the derivative
        # then you have found a landmark's left edge. See Scan0.png for visual reference.
        if(scan_derivative[i] < -jump):
            on_cylinder = True
            rays = 0
            sum_ray = 0.0
            sum_depth = 0
        # Each time you detect a landmark's right edge two consecutive values for the derivative
        # above the detection threshold appear. Only the first derivative value is valid and is
        # assocciated with the last suitable laser beam of the current analyzed scan.
        elif(scan_derivative[i] > jump and on_cylinder):
            on_cylinder = False
            # Add the values assocciated to the laser
            # beam that detects the landmark's right edge.
            rays += 1
            sum_ray += i
            sum_depth += scan[i]
            cylinder_list.append((sum_ray/rays, sum_depth/rays))
        if(on_cylinder and scan[i] > min_dist):
            rays += 1
            sum_ray += i
            sum_depth += scan[i]

    return cylinder_list

def compute_cartesian_coordinates(cylinders, cylinder_offset):
    result = []
    for c in cylinders:
        # --->>> Insert here the conversion from polar to Cartesian coordinates.
        # c is a tuple (beam_index, range).
        # For converting the beam index to an angle, use
        beam_angle = LegoLogfile.beam_index_to_angle(c[0])
        r = c[1] + cylinder_offset
        result.append( (r*cos(beam_angle), r*sin(beam_angle)) ) # Replace this by your (x,y)
    return result


if __name__ == '__main__':

    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Write a result file containing all cylinder records.
    # Format is: D C x[in mm] y[in mm] ...
    # With zero or more points.
    # Note "D C" is also written for otherwise empty lines (no
    # cylinders in scan)
    out_file = file("cylinders.txt", "w")
    for scan in logfile.scan_data:
        # Find cylinders.
        der = compute_derivative(scan, minimum_valid_distance)
        cylinders = find_cylinders(scan, der, depth_jump,
                                   minimum_valid_distance)
        cartesian_cylinders = compute_cartesian_coordinates(cylinders,
                                                            cylinder_offset)
        # Write to file.
        out_file.write("D C ")
        for c in cartesian_cylinders:
            out_file.write("%.1f %.1f " % c)
        out_file.write("\n")
    out_file.close()
