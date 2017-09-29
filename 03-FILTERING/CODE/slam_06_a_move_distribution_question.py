# Move a distribution (in x) by a given amount (an integer).
# 06_a_move_distribution
# Claus Brenner, 26 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    # --->>> Insert your code here.
    return Distribution(distribution.offset + delta, distribution.values)  # Replace this by your own result.

if __name__ == '__main__':
    # List of movements: move 3 times by 20.
    moves = [20, 20, 20]

    # Start with a known position: probability 1.0 at position 10.
    position = Distribution.triangle(10,2)
    plot(position.plotlists(0,100)[0], position.plotlists(0,100)[1],
         linestyle='steps')

    # Now move and plot.
    for m in moves:
        position = move(position, m)
        plot(position.plotlists(0,100)[0], position.plotlists(0,100)[1],
             linestyle='steps')
    ylim(0.0, 1.1)
    show()
