# For each cylinder in the scan, find its ray and depth.
# 03_c_find_cylinders
# Claus Brenner, 09 NOV 2012
from pylab import *
from lego_robot import *

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


if __name__ == '__main__':

    minimum_valid_distance = 20.0
    depth_jump = 100.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Pick one scan.
    scan = logfile.scan_data[8]

    # Find cylinders.
    der = compute_derivative(scan, minimum_valid_distance)
    cylinders = find_cylinders(scan, der, depth_jump, minimum_valid_distance)

    # Plot results.
    plot(scan)
    scatter([c[0] for c in cylinders], [c[1] for c in cylinders],
        c='r', s=200)
    show()
