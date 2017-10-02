# A* path planning.
# Addition of potential function keeps path away from obstacles.
# Note this modification changes only a single line in the A* algorithm.
# pp_01_f_astar_potential_function
# (c) Claus Brenner, 18 JAN 2014
from heapq import heappush, heappop
import numpy as np
import scipy.ndimage
import traceback
import gui
import common

# The world extents in units.
world_extents = (200, 150)

# The obstacle map.
# Obstacle = 255, free space = 0.
# Potential field = any value between 1 and 254
world_obstacles = np.zeros(world_extents, dtype=np.uint8)

# The array of visited cells during search.
visited_nodes = None

# Switch which determines if visited nodes shall be drawn in the GUI.
show_visited_nodes = True

# Switch which determines if potential function should be used.
use_potential_function = True

# The optimal path between start and goal. This is a list of (x,y) pairs.
optimal_path = []

# Functions for GUI functionality.
def add_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, True)
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path,
                           show_visited_nodes)
def remove_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, False)
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path,
                           show_visited_nodes)
def clear_obstacles():
    global world_obstacles
    world_obstacles = np.zeros(world_extents, dtype=np.uint8)
    update_callback()
def toggle_visited_nodes():
    global show_visited_nodes
    show_visited_nodes = not show_visited_nodes
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path,
                           show_visited_nodes)
def toggle_potential_function():
    global use_potential_function
    use_potential_function = not use_potential_function
    update_callback()
def apply_distance_transform():
    global world_obstacles
    if use_potential_function and np.max(world_obstacles) == 255:
        # Compute distance transform.
        dist_transform = 255-np.minimum(
            16*scipy.ndimage.morphology.distance_transform_edt(
            255-world_obstacles), 255)
        m = max(np.max(dist_transform), 1)  # Prevent m==0.
        world_obstacles = np.uint8((dist_transform * 255) / m)
    else:
        # Keep 255 values only (set all other to 0).
        world_obstacles = (world_obstacles == 255) * np.uint8(255)
def update_callback(pos = None):
    # First apply distance transform to world_obstacles.
    apply_distance_transform()
    # Call path planning algorithm.
    start, goal = gui.get_start_goal()
    if not (start==None or goal==None):
        global optimal_path
        global visited_nodes
        try:
            optimal_path, visited_nodes = astar(start, goal, world_obstacles)
        except Exception, e:
            print traceback.print_exc()
    # Draw new background.
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path,
                           show_visited_nodes)

# --------------------------------------------------------------------------
# A* algorithm.
# --------------------------------------------------------------------------

# Allowed movements and costs on the grid.
# Each tuple is: (movement_x, movement_y, cost).
s2 = np.sqrt(2)
movements = [ # Direct neighbors (4N).
              (1,0, 1.), (0,1, 1.), (-1,0, 1.), (0,-1, 1.),
              # Diagonal neighbors.
              # Comment this out to play with 4N only (faster).
              (1,1, s2), (-1,1, s2), (-1,-1, s2), (1,-1, s2),
            ]

def distance(p, q):
    """Helper function to compute distance between two points."""
    return np.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)

def astar(start, goal, obstacles):
    # In the beginning, the start is the only element in our front.
    # NOW, the first element is the total cost through the point, which is
    # the cost from start to point plus the estimated cost to the goal.
    # The second element is the cost of the path from the start to the point.
    # The third element is the position (cell) of the point.
    # The fourth component is the position we came from when entering the tuple
    # to the front.
    # Add the total cost of the start node as FIRST ELEMENT to the tuple.
    front = [ (0.001 + distance(start, goal), 0.001, start, None) ]

    # In the beginning, no cell has been visited.
    extents = obstacles.shape
    visited = np.zeros(extents, dtype=np.float32)

    # Also, we use a dictionary to remember where we came from.
    came_from = {}

    # While there are elements to investigate in our front.
    while front:
        # Get smallest item and remove from front.
        total_cost, cost, pos, previous = heappop(front)
        # total_cost, cost, pos, previous = min_cost_node

        # Check if this has been visited already.
        # visited[pos] is 0     --> node not visited
        # visited[pos] is not 0 --> node visited
        if(visited[pos] > 0):
            continue

        # Now it is visited. Mark with cost.
        visited[pos] = cost
        # Also remember that we came from previous when we marked pos.
        came_from[pos] = previous

        # Check if the goal has been reached.
        if pos == goal:
            break  # Finished!

        # Check all neighbors.
        for dx, dy, deltacost in movements:
            # Determine new position and check bounds.
            # - Compute new_x and new_y from old position 'pos' and dx, dy.
            new_x = pos[0] + dx
            new_y = pos[1] + dy

            # - Check that new_x is >= 0 and < extents[0], similarly for new_y.
            # - If not, skip the remaining part of this loop.
            if(new_x < 0 or new_x >= extents[0] or new_y < 0 or new_y >= extents[1]):
                continue

            # Add to front if: not visited before and no obstacle.
            new_pos = (new_x, new_y)
            # If visited is 0 and obstacles is not 255 (both at new_pos), then:
            # append the tuple (new_total_cost, new_cost, new_pos, pos) to the front.
            if((not visited[new_pos]) and obstacles[new_pos] != 255):
                # new_cost is the cost from start to new_pos plus the 'obstacle cost'.
                # obstacle cost =  obstacles[new_pos] / 64.
                # The divider 64 determines the tradeoff between 'avoiding obstacles' and
                # 'driving longer distances'. You may experiment with other values.
                # new_total_cost is new_cost plus the estimated cost from new_pos to goal.
                # Use heappush(). This will move the new tuple to the correct
                # location in the heap.
                new_cost = cost+deltacost+world_obstacles[new_pos]/64.0
                heappush(front, (new_cost+distance(new_pos, goal), new_cost, new_pos, pos))

    # Make sure to include the following code, which 'unwinds'
    # the path from goal to start, using the came_from dictionary.

    # Reconstruct path, starting from goal.
    path = []
    if pos == goal:  # If we reached the goal, unwind backwards.
        while pos:
            path.append(pos)
            pos = came_from[pos]
        path.reverse()  # Reverse so that path is from start to goal.

    return (path, visited)


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
    buttons = [("Clear", clear_obstacles),
               ("Use Potential Function", toggle_potential_function),
               ("Show Visited", toggle_visited_nodes)]

    # Init GUI.
    gui = gui.GUI(world_extents, 4, callbacks,
                  buttons, "on", "A* Algorithm using potential function.")

    # Start GUI main loop.
    gui.run()
