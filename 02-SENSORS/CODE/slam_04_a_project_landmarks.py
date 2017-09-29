# For each cylinder in the scan, find its cartesian coordinates,
# in the world coordinate system.
# Write the result to a file which contains all cylinders, for all scans.
# 04_a_project_landmarks
# Claus Brenner, 14 NOV 2012
from lego_robot import *
from slam_b_library import filter_step, compute_derivative,\
     find_cylinders, compute_cartesian_coordinates

# Put all cylinder extraction and position finding into one function.
def compute_scanner_cylinders(scan, jump, min_dist, cylinder_offset):
    der = compute_derivative(scan, min_dist)
    cylinders = find_cylinders(scan, der, jump, min_dist)
    scanner_cylinders = compute_cartesian_coordinates(cylinders, cylinder_offset)
    return scanner_cylinders

# Utility to write a list of cylinders to (one line of) a given file.
# Line header defines the start of each line, e.g. "D C" for a detected
# cylinder or "W C" for a world cylinder.
def write_cylinders(file_desc, line_header, cylinder_list):
    file_desc.write(line_header + " ")
    for c in cylinder_list:
        file_desc.write("%.1f %.1f " % c)
    file_desc.write("\n")

if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The constants we used for the cylinder detection in our scan.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Iterate over all positions.
    out_file = file("project_landmarks.txt", "w")
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

        # Write results to file.
        # The pose.
        out_file.write("F %f %f %f\n" % pose)
        # The detected cylinders in the scanner's coordinate system.
        write_cylinders(out_file, "D C", cartesian_cylinders)
        # The detected cylinders in the world coordinate system.
        write_cylinders(out_file, "W C", world_cylinders)

    out_file.close()
