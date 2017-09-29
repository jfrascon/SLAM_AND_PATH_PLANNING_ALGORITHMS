# Subsample the scan. For each point, find a closest point on the
# wall of the arena.
# From those point pairs, estimate a transform and apply this to the pose.
# 05_b_estimate_wall_transform
# Claus Brenner, 17 NOV 2012
from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import write_cylinders

from slam_04_d_apply_transform_question import\
     estimate_transform, apply_transform, correct_pose
     
from slam_05_a_find_wall_pairs_question import\
     get_subsampled_points, get_corresponding_points_on_wall


if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Iterate over all positions.
    out_file = file("estimate_wall_transform.txt", "w")
    for i in xrange(len(logfile.scan_data)):
        # Compute the new pose.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Subsample points.
        subsampled_points = get_subsampled_points(logfile.scan_data[i])
        world_points = [LegoLogfile.scanner_to_world(pose, c)
                        for c in subsampled_points]

        # Get the transformation
        left, right = get_corresponding_points_on_wall(world_points)
        trafo = estimate_transform(left, right, fix_scale = True)

        # Correct the initial position using trafo. Also transform points.
        if trafo:
            pose = correct_pose(pose, trafo)
            world_points = [apply_transform(trafo, p) for p in world_points]
        else:
            world_points = []

        # Write to file.
        # The pose.
        out_file.write("F %f %f %f\n" % pose)
        # Write the scanner points and corresponding points.
        write_cylinders(out_file, "W C", world_points)

    out_file.close()
