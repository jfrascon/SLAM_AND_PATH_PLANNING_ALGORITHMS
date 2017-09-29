# Plot a scan of the robot using matplotlib.
# 03_a_plot_scan
# Claus Brenner, 09 NOV 2012
from pylab import *
from lego_robot import *

# Read the logfile which contains all scans.
logfile = LegoLogfile()
logfile.read("robot4_scan.txt")

# Plot one scan.
plot(logfile.scan_data[8])
show()
