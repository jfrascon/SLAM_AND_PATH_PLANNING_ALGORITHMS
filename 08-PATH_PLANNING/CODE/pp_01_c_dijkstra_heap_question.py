# Dijkstra path planning.
# Third version: introduces a heap.
# pp_01_c_dijkstra_heap
# (c) Claus Brenner, 15 JAN 2014
from heapq import heappush, heappop
import numpy as np
import traceback
import gui
import common

# The world extents in units.
world_extents = (200, 150)

# The obstacle map.
# Obstacle = 255, free space = 0.
world_obstacles = np.zeros(world_extents, dtype=np.uint8)

# The array of visited cells during search.
visited_nodes = None

# The optimal path between start and goal. This is a list of (x,y) pairs.
optimal_path = []

# Functions for GUI functionality.
def add_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, True)
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path)
def remove_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, False)
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path)
def clear_obstacles():
    global world_obstacles
    world_obstacles = np.zeros(world_extents, dtype=np.uint8)
    update_callback()
def update_callback(pos = None):
    # Call path planning algorithm.
    start, goal = gui.get_start_goal()
    if not (start==None or goal==None):
        global optimal_path
        global visited_nodes
        try:
            optimal_path, visited_nodes = dijkstra(start, goal, world_obstacles)
        except Exception, e:
            print traceback.print_exc()
    # Draw new background.
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path)

# --------------------------------------------------------------------------
# Dijkstra algorithm.
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

def dijkstra(start, goal, obstacles):
    """Dijkstra's algorithm. Third version introduces a heap and is faster."""
    # In the beginning, the start is the only element in our front.
    # The first element is the cost of the path from the start to the point.
    # The second element is the position (cell) of the point.
    front = [ (0.001, start) ]

    # In the beginning, no cell has been visited.
    extents = obstacles.shape
    visited = np.zeros(extents, dtype=np.float32)

    # While there are elements to investigate in our front.
    while front:
        # Get smallest item from front and remove it from front.
        min_cost_node = heappop(front)

        # Check if this has been visited already. Skip the rest of the loop body if visited[pos] is > 0.
        cost, pos = min_cost_node
        if(visited[pos] > 0):
            continue

        # Now it is visited. Mark with the cost.
        visited[pos] = cost

        # Check if the goal has been reached.
        if pos == goal:
            break  # Finished!

        new_x = None
        new_y = None

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
            # append the tuple (cost + deltacost, new_pos) to the front.
            if(not visited[new_pos] and obstacles[new_pos] != 255):
                # Use heappush(). This will move the new tuple to the correct
                # location in the heap.
                heappush(front, (cost+deltacost, new_pos))

    return ([], visited)


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
    buttons = [("Clear", clear_obstacles)]

    # Init GUI.
    gui = gui.GUI(world_extents, 4, callbacks,
                  buttons, "on",
                  "Dijkstra Algorithm (now faster due to using a heap).")

    # Start GUI main loop.
    gui.run()
