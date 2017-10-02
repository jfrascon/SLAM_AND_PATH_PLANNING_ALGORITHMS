# Tkinter GUI used for playing with path planning algorithms.
# Author: Claus Brenner, 14 JAN 2014
from Tkinter import *
import PIL.Image as Image
import PIL.ImageTk as ImageTk
import numpy as np
from math import sin, cos, atan2

# For Tkinter error handler.
import traceback
import tkMessageBox

# Tkinter error handler (set below in main class).
def show_error(self, *args):
    err = traceback.format_exception(*args)
    tkMessageBox.showerror('Exception',err)

class GUI():
    def __init__(self, world_extents, display_factor, callbacks,
                 extra_buttons = None, start_goal_mode = None,
                 window_title = "Path Planning"):
        """Initializes the GUI stuff: canvas and buttons.

           Understands the following callbacks:
           'button_1_press', 'button_1_drag', 'button_1_release',
           'shift_button_1_press', 'shift_button_1_drag',
           'shift_button_1_release',
           all this variants for button 2 and 3, and 'update', which is called
           when start or goal are modified.
           extra_buttons is a list of (button_label, callback_function) pairs.
           Each entry will generate a button below the canvas.
           start_goal_mode is either None (can't set start/goal), 'on' (shows
           points only), or 'oriented' (shows point and orientation)."""
        # Set error handler.
        Tk.report_callback_exception = show_error

        # Setup GUI elements.
        self.world_extents = world_extents
        self.display_factor = display_factor
        self.extents = tuple(map(lambda v: v*self.display_factor,
                                 self.world_extents))
        self.root = Tk()
        self.root.title(window_title)
        self.frame1 = Frame(self.root)
        self.frame1.pack()
        self.world_canvas = Canvas(self.frame1,width=self.extents[0],
                                   height=self.extents[1], bg="black")
        self.world_canvas.pack()

        self.frame2 = Frame(self.root)
        self.frame2.pack()

        # Make optional buttons.
        for b in extra_buttons:
            button = Button(self.frame2, text=b[0], command=b[1])
            button.pack(side=LEFT, padx=2)

        # Make quit button.
        self.quit_button = Button(self.frame2, text="Quit", command=quit)
        self.quit_button.pack(side=LEFT, padx=2)

        # Button bindings.
        self.world_canvas.bind('<Button-1>', self.button_1_press)
        self.world_canvas.bind('<ButtonRelease-1>', self.button_1_release)
        self.world_canvas.bind('<Shift-Button-1>', self.shift_button_1_press)
        self.world_canvas.bind('<Button-2>', self.button_2_press)
        self.world_canvas.bind('<ButtonRelease-2>', self.button_2_release)
        self.world_canvas.bind('<Shift-Button-2>', self.shift_button_2_press)
        self.world_canvas.bind('<Button-3>', self.button_3_press)
        self.world_canvas.bind('<ButtonRelease-3>', self.button_3_release)
        self.world_canvas.bind('<Shift-Button-3>', self.shift_button_3_press)

        # Mousemove binding.
        self.world_canvas.bind('<Motion>', self.mousemove)

        # Init for button 1 & 2.
        self.button_pressed = [False, False, False]
        self.button_shifted = [False, False, False]

        # No background object.
        self.background_image_id = None

        # Set callbacks.
        self.callbacks = callbacks

        # Start and goal are undefined.
        self.start_goal_coordinates = [None, None]
        self.start_goal_canvas_ids = [None, None]

        # Install internal handlers of start and goal objects.
        self.start_goal_mode = start_goal_mode
        if start_goal_mode:
            self.callbacks["shift_button_1_press"] = self.place_start
            self.callbacks["shift_button_1_drag"] = self.drag_start
            # When released, signal main program by calling update (if this
            # callback has been set by the main program).
            if "update" in self.callbacks:
                self.callbacks["shift_button_1_release"] = \
                    self.callbacks["update"]
            # Same for the goal button.
            # Note: use button 2 and 3 as goal button to make it work
            # on different plattforms.
            self.callbacks["shift_button_2_press"] = self.place_goal
            self.callbacks["shift_button_3_press"] = self.place_goal
            self.callbacks["shift_button_2_drag"] = self.drag_goal
            self.callbacks["shift_button_3_drag"] = self.drag_goal
            if "update" in self.callbacks:
                self.callbacks["shift_button_2_release"] = \
                    self.callbacks["update"]
                self.callbacks["shift_button_3_release"] = \
                    self.callbacks["update"]

    def to_world(self, coord):
        """Transform display coordinates to world."""
        if coord:
            return (coord[0] / self.display_factor,
                    (self.extents[1]-coord[1]-1) / self.display_factor)
        else:
            return None

    def to_display(self, coord):
        """Transform display coordinates to world."""
        if coord:
            return (coord[0] * self.display_factor,
                    self.extents[1]-coord[1]*self.display_factor-1)
        else:
            return None

    def get_start_goal(self):
        """Return start and goal in lower left corner RHS coord system. May
           also return None for one or both, if not set."""
        if self.start_goal_mode == 'oriented':
            l = []
            for c in self.start_goal_coordinates:
                if c:
                    l.append( tuple(list(self.to_world(c[0:2]))+[c[2]]) )
                else:
                    l.append( None )
            return l
        else:
            return map(self.to_world, self.start_goal_coordinates)

    def set_background(self, np_array, color = False):
        """Takes a (numpy) array and sets this as background."""
        if color:
            img = Image.fromarray(np.flipud(np.uint8(np_array)), mode="RGB")
        else:
            img = Image.fromarray(np_array)
        if self.background_image_id:
            self.world_canvas.delete(self.background_image_id)
        img = img.resize(self.extents, Image.NEAREST)
        self.background_image = ImageTk.PhotoImage(img)
        self.background_id = self.world_canvas.create_image(0, 0,
            image=self.background_image, anchor=NW, tag="background")
        # Make sure drawing order is correct.
        self.set_display_order()

    def set_path(self, path, color = "white"):
        """Add a path to plot. If path is empty or None, delete old path."""
        self.world_canvas.delete("path")
        if path:
            disp_path = [ self.to_display(p) for p in path ]
            i = self.world_canvas.create_line(disp_path, tag="path",
                                              fill=color, width = 3)
            # Make sure drawing order is correct.
            self.set_display_order()

    def set_display_order(self):
        self.world_canvas.tag_lower("start")
        self.world_canvas.tag_lower("goal")
        self.world_canvas.tag_lower("path")
        self.world_canvas.tag_lower("background")

    def run(self):
        """Enter the main loop (will not return)."""
        self.root.mainloop()
        self.root.destroy()

    # Button press/ drag/ release logic.
    # Makes sure that drag events are only sent if a button is currently
    # pressed, and that shift-click drag events are sent even if the
    # user stops pressing the shift key while still pressing the mouse key.
    # The coordinates are converted to world coordinates.
    def button_press(self, button, shift, event):
        self.button_pressed[button] = True
        self.button_shifted[button] = shift
        if shift:
            callback = "shift_"
        else:
            callback = ""
        callback += "button_" + str(button+1) + "_press"
        if callback in self.callbacks:
            self.callbacks[callback](self.to_world((event.x, event.y)))
    def button_1_press(self, event):
        self.button_press(0, False, event)
    def shift_button_1_press(self, event):
        self.button_press(0, True, event)
    def button_2_press(self, event):
        self.button_press(1, False, event)
    def shift_button_2_press(self, event):
        self.button_press(1, True, event)
    def button_3_press(self, event):
        self.button_press(2, False, event)
    def shift_button_3_press(self, event):
        self.button_press(2, True, event)

    def button_release(self, button, event):
        self.button_pressed[button] = False
        if self.button_shifted[button]:
            callback = "shift_"
        else:
            callback = ""
        callback += "button_" + str(button+1) + "_release"
        try:
            self.callbacks[callback](self.to_world((event.x, event.y)))
        except:
            pass
    def button_1_release(self, event):
        self.button_release(0, event)
    def button_2_release(self, event):
        self.button_release(1, event)
    def button_3_release(self, event):
        self.button_release(2, event)

    def mousemove(self, event):
        # Only interested in drag.
        for i in [0,1,2]:
            if self.button_pressed[i]:
                if self.button_shifted[i]:
                    callback = "shift_"
                else:
                    callback = ""
                callback += "button_" + str(i+1) + "_drag"
                try:
                    self.callbacks[callback](self.to_world((event.x, event.y)))
                except:
                    pass

    # Logic for placing start and goal points on canvas.
    # Unfortunately, since these are "redirected" callbacks, we have to back
    # convert to display coordinates.
    def place_start(self, pos):
        self.place_start_goal(0, pos)
    def drag_start(self, pos):
        self.drag_start_goal(0, pos)

    def place_goal(self, pos):
        self.place_start_goal(1, pos)
    def drag_goal(self, pos):
        self.drag_start_goal(1, pos)

    def place_start_goal(self, start_goal, pos):
        self.set_start_goal(start_goal, self.to_display(pos), 0.0)

    def drag_start_goal(self, start_goal, pos):
        x, y = self.to_display(pos)
        old_x, old_y = self.start_goal_coordinates[start_goal][0:2]
        dx = x - old_x
        dy = y - old_y
        theta = atan2(-dy, dx)
        self.set_start_goal(start_goal, (old_x, old_y), theta)

    def set_start_goal(self, start_goal, pos, theta):
        x, y = pos
        if not (0 <= x < self.extents[0] and 0 <= y < self.extents[1]):
            return
        self.start_goal_coordinates[start_goal] = (x, y, theta)

        # Delete old ids used to draw start or goal.
        if self.start_goal_canvas_ids[start_goal]:
            for element in self.start_goal_canvas_ids[start_goal]:
                self.world_canvas.delete(element)
        self.start_goal_canvas_ids[start_goal] = []

        # Draw new start or goal.
        colors = ["yellow", "magenta"]
        radius = 6
        self.start_goal_canvas_ids[start_goal].append(
            self.world_canvas.create_oval(x-radius, y-radius,
                                          x+radius, y+radius,
                                          outline = colors[start_goal],
                                          width = 2,
                                          tag = "start"))
        if self.start_goal_mode == "oriented":
            self.start_goal_canvas_ids[start_goal].append(
                self.world_canvas.create_line(x, y,
                                              x + 3*radius*cos(theta),
                                              y - 3*radius*sin(theta),
                                              fill = colors[start_goal],
                                              width = 2,
                                              tag = "goal"))

        # Make sure drawing order is correct.
        self.set_display_order()
