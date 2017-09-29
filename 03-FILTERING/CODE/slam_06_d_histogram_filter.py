# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)

def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""

    # --->>> Put your code here.
    len_a = len(a.values)
    len_tot = len_a + len(b.values) - 1
    values = [0] * len_tot

    a_offset = a.start()
    b_offset = b.start()
    offset = b_offset + a_offset

    for i in xrange(0, len_tot):
        for j in xrange(i, -1, -1):
            values[i] +=  a.value(i-j + a.start()) * b.value(j + b.start())

    d = Distribution(offset, values)
    d.normalize()

    return d  # Replace this by your own result.

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
    arena = (0,220)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 10

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)

    # This is the filter loop.
    for i in xrange(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(controls[i], 10)
        position = convolve(position, control)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b', linestyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        position = multiply(position, measurement)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r', linestyle='steps')

    show()
