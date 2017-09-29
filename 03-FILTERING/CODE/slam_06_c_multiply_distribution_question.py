# Multiply a distribution by another distribution.
# 06_c_multiply_distribution
# Claus Brenner, 26 NOV 2012
from pylab import plot, show
from distribution import *

def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""

    # --->>> Put your code here.
    offset = max(a.start(), b.start())
    final_index = min(a.start() + len(a.values) - 1, b.start() + len(b.values) - 1)

    # if offset < final_index then there is overlapping between the functions.
    if offset > final_index: # there isn't overlapping between the functions.
        offset = 0
        values = [0]
        d = Distribution(offset, values)
        return d

    values = [0] * (final_index - offset + 1) # length = final_index - offset + 1

    for i in xrange(offset, final_index + 1):
        values[i - offset] = a.value(i) * b.value(i)

    d = Distribution(offset, values)
    d.normalize()
    return d  # Modify this to return your result.


if __name__ == '__main__':
    arena = (0,1000)

    # Here is our assumed position. Plotted in blue.
    position_value = 400
    position_error = 100
    position = Distribution.triangle(position_value, position_error)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         color='b', linestyle='steps')

    # Here is our measurement. Plotted in green.
    # That is what we read from the instrument.
    measured_value = 550
    measurement_error = 200
    measurement = Distribution.triangle(measured_value, measurement_error)
    plot(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
         color='g', linestyle='steps')

    # Now, we integrate our sensor measurement. Result is plotted in red.
    position_after_measurement = multiply(position, measurement)
    plot(position_after_measurement.plotlists(*arena)[0],
         position_after_measurement.plotlists(*arena)[1],
         color='r', linestyle='steps')

    show()
