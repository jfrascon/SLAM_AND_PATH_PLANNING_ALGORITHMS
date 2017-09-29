# Python routines useful for handling ikg's LEGO robot data.
# Author: Claus Brenner, 28.10.2012
from math import sin, cos, pi

# In previous versions, the S record included the number of scan points.
# If so, set this to true.
s_record_has_count = True

# Class holding log data of our Lego robot.
# The logfile understands the following records:
# P reference position (of the robot)
# S scan data
# I indices of poles in the scan data (determined by an external algorithm)
# M motor (ticks from the odometer) data
# F filtered data (robot position, or position and heading angle)
# L landmark (reference landmark, fixed)
# D detected landmark, in the scanner's coordinate system. C is cylinders.
# W something to draw in the world coordinate system. C is cylinders.
#
class LegoLogfile(object):
    def __init__(self):
        self.reference_positions = []
        self.scan_data = []
        self.pole_indices = []
        self.motor_ticks = []
        self.filtered_positions = []
        self.landmarks = []
        self.detected_cylinders = []
        self.world_cylinders = []
        self.last_ticks = None

    def read(self, filename):
        """Reads log data from file. Calling this multiple times with different
           files will result in a merge of the data, i.e. if one file contains
           M and S data, and the other contains M and P data, then LegoLogfile
           will contain S from the first file and M and P from the second file."""
        # If information is read in repeatedly, replace the lists instead of appending,
        # but only replace those lists that are present in the data.
        first_reference_positions = True
        first_scan_data = True
        first_pole_indices = True
        first_motor_ticks = True
        first_filtered_positions = True
        first_landmarks = True
        first_detected_cylinders = True
        first_world_cylinders = True
        f = open(filename)
        for l in f:
            sp = l.split()
            # P is the reference position.
            # File format: P timestamp[in ms] x[in mm] y[in mm]
            # Stored: A list of tuples [(x, y), ...] in reference_positions.
            if sp[0] == 'P':
                if first_reference_positions:
                    self.reference_positions = []
                    first_reference_positions = False 
                self.reference_positions.append( (int(sp[2]), int(sp[3])) )

            # S is the scan data.
            # File format:
            #  S timestamp[in ms] distances[in mm] ...
            # Or, in previous versions (set s_record_has_count to True):
            #  S timestamp[in ms] count distances[in mm] ...
            # Stored: A list of tuples [ [(scan1_distance,... ), (scan2_distance,...) ]
            #   containing all scans, in scan_data.
            elif sp[0] == 'S':
                if first_scan_data:
                    self.scan_data = []
                    first_scan_data = False
                if s_record_has_count:
                    self.scan_data.append(tuple(map(int, sp[3:])))
                else:
                    self.scan_data.append(tuple(map(int, sp[2:])))

            # I is indices of poles in the scan.
            # The indices are given in scan order (counterclockwise).
            # -1 means that the pole could not be clearly detected.
            # File format: I timestamp[in ms] index ...
            # Stored: A list of tuples of indices (including empty tuples):
            #  [(scan1_pole1, scan1_pole2,...), (scan2_pole1,...)...]
            elif sp[0] == 'I':
                if first_pole_indices:
                    self.pole_indices = []
                    first_pole_indices = False
                self.pole_indices.append(tuple(map(int, sp[2:])))

            # M is the motor data.
            # File format: M timestamp[in ms] pos[in ticks] tachoCount[in ticks] acceleration[deg/s^2] rotationSpeed[deg/s] ...
            #   (4 values each for: left motor, right motor, and third motor (not used)).
            # Stored: A list of tuples [ (inc-left, inc-right), ... ] with tick increments, in motor_ticks.
            # Note that the file contains absolute ticks, but motor_ticks contains the increments (differences).
            elif sp[0] == 'M':
                ticks = (int(sp[2]), int(sp[6]))
                if first_motor_ticks:
                    self.motor_ticks = []
                    first_motor_ticks = False
                    self.last_ticks = ticks
                self.motor_ticks.append(
                    tuple([ticks[i]-self.last_ticks[i] for i in range(2)]))
                self.last_ticks = ticks

            # F is filtered trajectory. No time stamp is used.
            # File format: F x[in mm] y[in mm]
            # OR:          F x[in mm] y[in mm] heading[in radians]
            # Stored: A list of tuples, each tuple is (x y) or (x y heading)
            elif sp[0] == 'F':
                if first_filtered_positions:
                    self.filtered_positions = []
                    first_filtered_positions = False
                self.filtered_positions.append( tuple( map(float, sp[1:])) )

            # L is landmark. This is actually background information, independent
            # of time.
            # File format: L <type> info...
            # Supported types:
            # Cylinder: L C x y diameter.
            # Stored: List of (<type> info) tuples.
            elif sp[0] == 'L':
                if first_landmarks:
                    self.landmarks = []
                    first_landmarks = False
                if sp[1] == 'C':
                    self.landmarks.append( tuple(['C'] + map(float, sp[2:])) )
                    
            # D is detected landmarks (in each scan).
            # File format: D <type> info...
            # Supported types:
            # Cylinder: D C x y x y ...
            #   Stored: List of lists of (x, y) tuples of the cylinder positions,
            #   one list per scan.
            elif sp[0] == 'D':
                if sp[1] == 'C':
                    if first_detected_cylinders:
                        self.detected_cylinders = []
                        first_detected_cylinders = False
                    cyl = map(float, sp[2:])
                    self.detected_cylinders.append([(cyl[2*i], cyl[2*i+1]) for i in range(len(cyl)/2)])

            # W is information to be plotted in the world (in each scan).
            # File format: W <type> info...
            # Supported types:
            # Cylinder: W C x y x y ...
            #   Stored: List of lists of (x, y) tuples of the cylinder positions,
            #   one list per scan.
            elif sp[0] == 'W':
                if sp[1] == 'C':
                    if first_world_cylinders:
                        self.world_cylinders = []
                        first_world_cylinders = False
                    cyl = map(float, sp[2:])
                    self.world_cylinders.append([(cyl[2*i], cyl[2*i+1]) for i in range(len(cyl)/2)])

        f.close()

    def size(self):
        """Return the number of entries. Take the max, since some lists may be empty."""
        return max(len(self.reference_positions), len(self.scan_data),
                   len(self.pole_indices), len(self.motor_ticks),
                   len(self.filtered_positions), len(self.detected_cylinders),
                   len(self.world_cylinders))

    @staticmethod
    def beam_index_to_angle(i, mounting_angle = -0.06981317007977318):
        """Convert a beam index to an angle, in radians."""
        return (i - 330.0) * 0.006135923151543 + mounting_angle

    @staticmethod
    def scanner_to_world(pose, point):
        """Given a robot pose (rx, ry, heading) and a point (x, y) in the
           scanner's coordinate system, return the point's coordinates in the
           world coordinate system."""
        dx = cos(pose[2])
        dy = sin(pose[2])
        x, y = point
        return (x * dx - y * dy + pose[0], x * dy + y * dx + pose[1])        

    def info(self, i):
        """Prints reference pos, number of scan points, and motor ticks."""
        s = ""
        if i < len(self.reference_positions):
            s += " | ref-pos: %4d %4d" % self.reference_positions[i]

        if i < len(self.scan_data):
            s += " | scan-points: %d" % len(self.scan_data[i])

        if i < len(self.pole_indices):
            indices = self.pole_indices[i]
            if indices:
                s += " | pole-indices:"
                for idx in indices:
                    s += " %d" % idx
            else:
                s += " | (no pole indices)"
                    
        if i < len(self.motor_ticks):
            s += " | motor: %d %d" % self.motor_ticks[i]

        if i < len(self.filtered_positions):
            f = self.filtered_positions[i]
            s += " | filtered-pos:"
            for j in range(len(f)):
                s += " %.1f" % f[j]

        return s
