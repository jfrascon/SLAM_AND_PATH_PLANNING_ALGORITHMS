# A* path planning using kinematic state space of car.
# pp_02_b_car_statespace_astar_solution
# (c) Claus Brenner, 16 JAN 2014
import numpy as np
from math import sqrt, sin, cos, pi, floor, radians, copysign
from heapq import heappush, heappop
import traceback
import gui
import common

# The world extents in units.
world_extents = (100, 75)

# The obstacle map.
# Obstacle = 255, free space = 0.
world_obstacles = np.zeros(world_extents, dtype=np.uint8)

# The array of visited cells during search.
visited_cells = None

# Switch which determines if visited cells shall be drawn in the GUI.
show_visited_cells = True

# The optimal path between start and goal. This is a list of (x,y) pairs.
optimal_path = []

# Functions for GUI functionality.
def add_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, True)
    common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
                           show_visited_cells)
def remove_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, False)
    common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
                           show_visited_cells)
def clear_obstacles():
    global world_obstacles
    world_obstacles = np.zeros(world_extents, dtype=np.uint8)
    update_callback()
def toggle_visited_cells():
    global show_visited_cells
    show_visited_cells = not show_visited_cells
    common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
                           show_visited_cells)
def toggle_movement():
    global max_movement_id
    max_movement_id = 9 - max_movement_id  # Toggles between 3 and 6.
    update_callback()
def update_callback(pos = None):
    # Call path planning algorithm.
    start, goal = gui.get_start_goal()
    if not (start==None or goal==None):
        global optimal_path
        global visited_cells
        try:
            optimal_path, visited_cells = \
                astar_statespace(start, goal, world_obstacles)
        except Exception, e:
            print traceback.print_exc()
    # Draw new background.
    common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
                           show_visited_cells)

# --------------------------------------------------------------------------
# Helper class for curved segments.
# --------------------------------------------------------------------------
class CurveSegment:
    @staticmethod
    def end_pose(start_pose, curvature, length):
        """Returns end pose, given start pose, curvature and length."""
        x, y, theta = start_pose
        if curvature == 0.0:
            # Linear movement.
            x += length * cos(theta)
            y += length * sin(theta)
            return (x, y, theta)
        else:
            # Curve segment of radius 1/curvature.
            tx = cos(theta)
            ty = sin(theta)
            radius = 1.0/curvature
            xc = x - radius * ty  # Center of circle.
            yc = y + radius * tx
            angle = length / radius
            cosa = cos(angle)
            sina = sin(angle)
            nx = xc + radius * (cosa * ty + sina * tx)
            ny = yc + radius * (sina * ty - cosa * tx)
            ntheta = (theta + angle + pi) % (2*pi) - pi
            return (nx, ny, ntheta)

    @staticmethod
    def segment_points(start_pose, curvature, length, delta_length):
        """Return points of segment, at delta_length intervals."""
        l = 0.0
        delta_length = copysign(delta_length, length)
        points = []
        while abs(l) < abs(length):
            points.append(CurveSegment.end_pose(start_pose, curvature, l)[0:2])
            l += delta_length
        return points

# --------------------------------------------------------------------------
# Exploration of car's kinematic state space.
# --------------------------------------------------------------------------

# Allowed movements. These are given as tuples: (curvature, length).
# The list contains forward and backward movements.
movements = [(1.0/10, 5.0), (0.0, 5.0), (-1.0/10, 5.0),
             (1.0/10, -5.0), (0.0, -5.0), (-1.0/10, -5.0)]
max_movement_id = 3  # 3 for forward only, 6 for forward and backward.

# Helper functions.
def distance(p, q):
    """Return Euclidean distance between two points."""
    return sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)

def states_close(p, q):
    """Checks if two poses (x, y, heading) are the same within a tolerance."""
    d_angle = abs((p[2]-q[2]+pi) % (2*pi) - pi)
    # For the sake of simplicity, tolerances are hardcoded here:
    # 15 degrees for the heading angle, 2.0 for the position.
    return d_angle < radians(15.) and distance(p, q) <= 2.0

def pose_index(pose):
    """Given a pose, returns a discrete version (a triple index)."""
    # Again, raster sizes are hardcoded here for simplicity.
    pos_raster = 1.0
    heading_raster = radians(10.)
    xi = int(floor(pose[0] / pos_raster))
    yi = int(floor(pose[1] / pos_raster))
    ti = int(floor(pose[2] / heading_raster))
    return (xi, yi, ti)


def astar_statespace(start_pose, goal_pose, obstacles):
    """Try to find a sequence of curve segments from start to goal."""

    # Init front: the only element is the start pose.
    # Each tuple contains:
    # (total_cost, cost, pose, previous_pose, move_index).
    front = [ (0.0001 + distance(start_pose, goal_pose), 0.0001, start_pose, None, None) ]

    # In the beginning, no cell has been visited.
    extents = obstacles.shape
    visited_cells = np.zeros(extents, dtype=np.float32)

    # Also, no states have been generated.
    generated_states = {}

    while front:
        # Stop search if the front gets too large.
        if len(front) > 500000:
            print "Timeout."
            break

        # Pop smallest item from heap.
        total_cost, cost, pose, previous_pose, move = heappop(front)

        # Compute discrete index: compute a discrete pose index, pose_idx, from pose.
        pose_idx = pose_index(pose)

        # Check if this has been visited already: check if pose_index is in generated_states already,
        # and if so, skip the rest of the loop. (This is the point where we prevent exponential growth.)
        if(pose_idx in generated_states):
            continue

        # Mark visited_cell which encloses this pose.
        visited_cells[int(pose[0]), int(pose[1])] = cost

        # Enter into visited states, also use this to remember where we
        # came from and which move we used.
        # Change the following line so that the index is the discrete pose instead of the
        # continuous pose.
        generated_states[pose_idx] = (previous_pose, move)

        # Check if we have (approximately) reached the goal.
        if states_close(pose, goal_pose):
            break  # Finished!

        # Check all possible movements.
        for i in xrange(max_movement_id):
            curvature, length = movements[i]

            # Determine new pose and check bounds.
            new_pose = CurveSegment.end_pose(pose, curvature, length)
            if not (0 <= new_pose[0] < extents[0] and \
                    0 <= new_pose[1] < extents[1]):
                continue

            # Add to front if there is no obstacle.
            if not obstacles[(int(new_pose[0]), int(new_pose[1]))] == 255:
                new_cost = cost + abs(length)
                total_cost = new_cost + distance(new_pose, goal_pose)
                heappush(front, (total_cost, new_cost, new_pose, pose, i))

    # Reconstruct path, starting from goal.
    # (Take this part of the code as is. Note it is different from the
    #  pp_02_a code.)
    if states_close(pose, goal_pose):
        path = []
        path.append(pose[0:2])
        pose, move = generated_states[pose_index(pose)]
        while pose:
            points = CurveSegment.segment_points(pose,
                movements[move][0], movements[move][1], 2.0)
            path.extend(reversed(points))
            pose, move = generated_states[pose_index(pose)]
        path.reverse()
    else:
        path = []

    return path, visited_cells


# Main program.
if __name__ == '__main__':
    # Link functions to buttons.
    callbacks = {"update": update_callback,
                 "button_1_press": add_obstacle,
                 "button_1_drag": add_obstacle,
                 "button_1_release": update_callback,
                 "button_2_press": remove_obstacle,
                 "button_2_drag": remove_obstacle,
                 "button_2_release": update_callback,
                 "button_3_press": remove_obstacle,
                 "button_3_drag": remove_obstacle,
                 "button_3_release": update_callback,
                 }
    # Extra buttons.
    buttons = [("Forward/FW+BW", toggle_movement),
               ("Clear", clear_obstacles),
               ("Show Visited", toggle_visited_cells)]

    # Init GUI.
    gui = gui.GUI(world_extents, 8, callbacks,
                  buttons, "oriented",
                  "Car state space A*.")

    # Start GUI main loop.
    gui.run()
