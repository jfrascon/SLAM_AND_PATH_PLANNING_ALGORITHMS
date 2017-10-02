# Helper routines for the path planning algorithms.
# These were pulled out to keep the main implementation functions clear.
# Author: Claus Brenner, 17 JAN 2014
from __future__ import print_function
import numpy as np

def set_obstacle(obstacle_array, pos, on):
    """Draw an obstacle (2N+1 x 2N+1) box into numpy obstacle_array."""
    N = 2  # Box size will be 2N+1
    # Handle borders.
    extents = obstacle_array.shape
    x, y = pos
    l = max(0, x-N)
    r = min(extents[0]-1, x+N)
    d = max(0, y-N)
    u = min(extents[1]-1, y+N)
    if l<=r and d<=u:
        if on:
            mask = np.ones((r-l+1, u-d+1)) * 255
        else:
            mask = np.zeros((r-l+1, u-d+1))
        global world_obstacles
        #==>
        #print(l,r,d,u)
        #<==
        obstacle_array[l:r+1, d:u+1] = mask

def draw_background(gui, obstacle_array, visited_array, path,
                    show_visited = True):
    """Set background bitmap and path."""
    # This is intended to be used while the
    # user is actively drawing/ undrawing the background.
    # Red is all obstacle values which are exactly 255.
    red = (obstacle_array == 255) * np.uint8(255)
    # Blue shows obstacle values from 0..254.
    # If red (I think is obstacle_array) is 0 and 255 only, blue will be all zero.
    blue = obstacle_array - red
    if not show_visited or visited_array is None:
        green = np.zeros(obstacle_array.shape, dtype=np.uint8)
    else:
        max_dist = np.amax(visited_array)
        if max_dist > 0:
            green = np.uint8(visited_array * (255.0 / max_dist))
        else:
            green = np.zeros(obstacle_array.shape, dtype=np.uint8)
    gui.set_background(np.dstack((red.T, green.T, blue.T)), color=True)
    gui.set_path(path)
