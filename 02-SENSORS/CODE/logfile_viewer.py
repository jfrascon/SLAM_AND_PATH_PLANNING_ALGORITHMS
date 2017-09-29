# Python routines to inspect a ikg LEGO robot logfile.
# Author: Claus Brenner, 28 OCT 2012
from Tkinter import *
import tkFileDialog
from lego_robot import *
from math import sin, cos, pi

# The canvas and world extents of the scene.
# Canvas extents in pixels, world extents in millimeters.
canvas_extents = (600, 600)
world_extents = (2000.0, 2000.0)

# The extents of the sensor canvas.
sensor_canvas_extents = canvas_extents

# The maximum scanner range used to scale scan measurement drawings,
# in millimeters.
max_scanner_range = 2200.0

class DrawableObject(object):
    def draw(self, at_step):
        print "To be overwritten - will draw a certain point in time:", at_step

    def background_draw(self):
        print "Background draw."

class Trajectory(DrawableObject):
    def __init__(self, points, canvas,
                 point_size2 = 2, background_color = "gray", cursor_color = "red"):
        self.points = points
        self.canvas = canvas
        self.point_size2 = point_size2
        self.background_color = background_color
        self.cursor_color = cursor_color
        self.cursor_object = None
        self.cursor_object2 = None

    def background_draw(self):
        if self.points:
            p_xy_only = []
            for p in self.points:
                self.canvas.create_oval(\
                    p[0]-self.point_size2, p[1]-self.point_size2,
                    p[0]+self.point_size2, p[1]+self.point_size2,
                    fill=self.background_color, outline="")
                p_xy_only.append(p[0:2])
            self.canvas.create_line(*p_xy_only, fill=self.background_color)

    def draw(self, at_step):
        if self.cursor_object:
            self.canvas.delete(self.cursor_object)
            self.cursor_object = None
            self.canvas.delete(self.cursor_object2)
            self.cursor_object2 = None
        if at_step < len(self.points):
            p = self.points[at_step]
            self.cursor_object = self.canvas.create_oval(\
                p[0]-self.point_size2-1, p[1]-self.point_size2-1,
                p[0]+self.point_size2+1, p[1]+self.point_size2+1,
                fill=self.cursor_color, outline="")
            if len(p) > 2:
                self.cursor_object2 = self.canvas.create_line(p[0], p[1],
                    p[0] + cos(p[2]) * 50,
                    p[1] - sin(p[2]) * 50,
                    fill = self.cursor_color)

class ScannerData(DrawableObject):
    def __init__(self, list_of_scans, canvas, canvas_extents, scanner_range):
        self.canvas = canvas
        self.canvas_extents = canvas_extents
        self.cursor_object = None

        # Convert polar scanner measurements into xy form, in canvas coords.
        # Store the result in self.scan_polygons.
        self.scan_polygons = []
        for s in list_of_scans:
            poly = [ to_sensor_canvas((0,0), canvas_extents, scanner_range) ]
            i = 0
            for m in s:
                angle = LegoLogfile.beam_index_to_angle(i)
                x = m * cos(angle)
                y = m * sin(angle)
                poly.append(to_sensor_canvas((x,y), canvas_extents, scanner_range))
                i += 1
            poly.append(to_sensor_canvas((0,0), canvas_extents, scanner_range))
            self.scan_polygons.append(poly)

    def background_draw(self):
        # Draw x axis.
        self.canvas.create_line(
            self.canvas_extents[0]/2, self.canvas_extents[1]/2,
            self.canvas_extents[0]/2, 20,
            fill="black")
        self.canvas.create_text(
            self.canvas_extents[0]/2 + 10, 20, text="x" )
        # Draw y axis.
        self.canvas.create_line(
            self.canvas_extents[0]/2, self.canvas_extents[1]/2,
            20, self.canvas_extents[1]/2,
            fill="black")
        self.canvas.create_text(
            20, self.canvas_extents[1]/2 - 10, text="y" )
        # Draw big disk in the scan center.
        self.canvas.create_oval(
            self.canvas_extents[0]/2-20, self.canvas_extents[1]/2-20,
            self.canvas_extents[0]/2+20, self.canvas_extents[1]/2+20,
            fill="gray", outline="")

    def draw(self, at_step):
        if self.cursor_object:
            self.canvas.delete(self.cursor_object)
            self.cursor_object = None
        if at_step < len(self.scan_polygons):
            self.cursor_object = self.canvas.create_polygon(self.scan_polygons[at_step], fill="blue")

class Landmarks(DrawableObject):
    # In contrast other classes, Landmarks stores the original world coords and
    # transforms them when drawing.
    def __init__(self, landmarks, canvas, canvas_extents, world_extents, color = "gray"):
        self.landmarks = landmarks
        self.canvas = canvas
        self.canvas_extents = canvas_extents
        self.world_extents = world_extents
        self.color = color

    def background_draw(self):
        for l in self.landmarks:
            if l[0] =='C':
                x, y = l[1:3]
                ll = to_world_canvas((x - l[3], y - l[3]), self.canvas_extents, self.world_extents)
                ur = to_world_canvas((x + l[3], y + l[3]), self.canvas_extents, self.world_extents)
                self.canvas.create_oval(ll[0], ll[1], ur[0], ur[1], fill=self.color)

    def draw(self, at_step):
        # Landmarks are background only.
        pass
    
class Points(DrawableObject):
    def __init__(self, points, canvas, color = "red", radius = 5):
        self.points = points
        self.canvas = canvas
        self.color = color
        self.radius = radius
        self.cursor_objects = []

    def background_draw(self):
        pass

    def draw(self, at_step):
        if self.cursor_objects:
            map(self.canvas.delete, self.cursor_objects)
            self.cursor_objects = []
        if at_step < len(self.points):
            for c in self.points[at_step]:
                self.cursor_objects.append(self.canvas.create_oval(
                    c[0]-self.radius, c[1]-self.radius,
                    c[0]+self.radius, c[1]+self.radius,
                    fill=self.color))

# World canvas is x right, y up, and scaling according to canvas/world extents.
def to_world_canvas(world_point, canvas_extents, world_extents):
    """Transforms a point from world coord system to world canvas coord system."""
    x = int(world_point[0] / world_extents[0] * canvas_extents[0])
    y = int(canvas_extents[1] - 1 - world_point[1] / world_extents[1] * canvas_extents[1])
    return (x, y)

# Sensor canvas is "in driving direction", with x up, y left, (0,0) in the center
# and scaling according to canvas_extents and max_scanner_range.
def to_sensor_canvas(sensor_point, canvas_extents, scanner_range):
    """Transforms a point from sensor coordinates to sensor canvas coord system."""
    scale = canvas_extents[0] / 2.0 / scanner_range
    x = int(canvas_extents[0] / 2.0 - sensor_point[1] * scale)
    y = int(canvas_extents[1] / 2.0 - 1 - sensor_point[0] * scale)
    return (x, y)

def slider_moved(index):
    """Callback for moving the scale slider."""
    i = int(index)
    # Call all draw objects.
    for d in draw_objects:
        d.draw(i)

    # Print info about current point.
    info.config(text=logfile.info(i))

def add_file():
    filename = tkFileDialog.askopenfilename(filetypes = [("all files", ".*"), ("txt files", ".txt")])
    if filename:
        # If the file is in the list already, remove it (so it will be appended
        # at the end).
        if filename in all_file_names:
            all_file_names.remove(filename)
        all_file_names.append(filename)
        load_data()

def load_data():
    global canvas_extents, sensor_canvas_extents, world_extents, max_scanner_range
    for filename in all_file_names:
        logfile.read(filename)

    global draw_objects
    draw_objects = []
    scale.configure(to=logfile.size()-1)

    # Insert: landmarks.
    draw_objects.append(Landmarks(logfile.landmarks, world_canvas, canvas_extents, world_extents))

    # Insert: reference trajectory.
    positions = [to_world_canvas(pos, canvas_extents, world_extents) for pos in logfile.reference_positions]
    draw_objects.append(Trajectory(positions, world_canvas,
        cursor_color="red", background_color="#FFB4B4"))

    # Insert: filtered trajectory.
    if logfile.filtered_positions:
        if len(logfile.filtered_positions[0]) > 2:
            positions = [tuple(list(to_world_canvas(pos, canvas_extents, world_extents)) + [pos[2]]) for pos in logfile.filtered_positions]
        else:
            positions = [to_world_canvas(pos, canvas_extents, world_extents) for pos in logfile.filtered_positions]
        draw_objects.append(Trajectory(positions, world_canvas,
            cursor_color="blue", background_color="lightblue"))

    # Insert: scanner data.
    draw_objects.append(ScannerData(logfile.scan_data, sensor_canvas,
        sensor_canvas_extents, max_scanner_range))

    # Insert: detected cylinders, in scanner coord system.
    if logfile.detected_cylinders:
        positions = [[to_sensor_canvas(pos, sensor_canvas_extents, max_scanner_range)
                     for pos in cylinders_one_scan ]
                     for cylinders_one_scan in logfile.detected_cylinders ]
        draw_objects.append(Points(positions, sensor_canvas, "#88FF88"))

    # Insert: detected cylinders, transformed into world coord system.
    if logfile.detected_cylinders and logfile.filtered_positions and \
        len(logfile.filtered_positions[0]) > 2:
        positions = []
        for i in xrange(min(len(logfile.detected_cylinders), len(logfile.filtered_positions))):
            this_pose_positions = []
            pos = logfile.filtered_positions[i]
            dx = cos(pos[2])
            dy = sin(pos[2])
            for pole in logfile.detected_cylinders[i]:
                x = pole[0] * dx - pole[1] * dy + pos[0]
                y = pole[0] * dy + pole[1] * dx + pos[1]
                p = to_world_canvas((x,y), canvas_extents, world_extents)
                this_pose_positions.append(p)
            positions.append(this_pose_positions)
        draw_objects.append(Points(positions, world_canvas, "#88FF88"))

    # Insert: world objects, cylinders.
    if logfile.world_cylinders:
        positions = [[to_world_canvas(pos, canvas_extents, world_extents)
                      for pos in cylinders_one_scan]
                      for cylinders_one_scan in logfile.world_cylinders]
        draw_objects.append(Points(positions, world_canvas, "#DC23C5"))

    # Start new canvas and do all background drawing.
    world_canvas.delete(ALL)
    sensor_canvas.delete(ALL)
    for d in draw_objects:
        d.background_draw()


# Main program.
if __name__ == '__main__':

    # Construct logfile (will be read in load_data()).
    logfile = LegoLogfile()

    # Setup GUI stuff.
    root = Tk()
    frame1 = Frame(root)
    frame1.pack()
    world_canvas = Canvas(frame1,width=canvas_extents[0],height=canvas_extents[1],bg="white")
    world_canvas.pack(side=LEFT)
    sensor_canvas = Canvas(frame1,width=sensor_canvas_extents[0],height=sensor_canvas_extents[1],bg="white")
    sensor_canvas.pack(side=RIGHT)
    scale = Scale(root, orient=HORIZONTAL, command = slider_moved)
    scale.pack(fill=X)
    info = Label(root)
    info.pack()
    frame2 = Frame(root)
    frame2.pack()
    load = Button(frame2,text="Load (additional) logfile",command=add_file)
    load.pack(side=LEFT)
    reload_all = Button(frame2,text="Reload all",command=load_data)
    reload_all.pack(side=RIGHT)

    # The list of objects to draw.
    draw_objects = []

    # Ask for file.
    all_file_names = []
    add_file()

    root.mainloop()
    root.destroy()
