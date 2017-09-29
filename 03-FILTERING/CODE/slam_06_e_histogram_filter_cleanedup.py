# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_e_histogram_filter_cleanedup
# Claus Brenner, 29 NOV 2012
from distribution import *
from pylab import plot, show, ylim

# Import the helper functions from the previous file.
# If you implemented these functions in another file, put the filename here.
from slam_06_d_histogram_filter import move, convolve, multiply

#
# Helpers.
#
def histogram_plot(prediction, measurement, correction):
    """Helper to draw all curves in each filter step."""
    plot(prediction.plotlists(*arena)[0], prediction.plotlists(*arena)[1],
         color='#C0C0FF', linestyle='steps', linewidth=5)
    plot(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
         color='#C0FFC0', linestyle='steps', linewidth=5)
    plot(correction.plotlists(*arena)[0], correction.plotlists(*arena)[1],
         color='#FFC0C0', linestyle='steps', linewidth=5)

def histogram_control_plot(control, prediction, measurement, correction):
    """Helper to draw all curves in each filter step."""
    plot(control.plotlists(*arena)[0], control.plotlists(*arena)[1],
         color='#00FFFF', linestyle='steps', linewidth=5)
    plot(prediction.plotlists(*arena)[0], prediction.plotlists(*arena)[1],
         color='#C0C0FF', linestyle='steps', linewidth=5)
    plot(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
         color='#C0FFC0', linestyle='steps', linewidth=5)
    plot(correction.plotlists(*arena)[0], correction.plotlists(*arena)[1],
         color='#FFC0C0', linestyle='steps', linewidth=5)

#
# Histogram filter step.
#
def histogram_filter_step(belief, control, measurement):
    """Bayes filter step implementation: histogram filter."""
    # These two lines is the entire filter!
    prediction = convolve(belief, control)
    correction = multiply(prediction, measurement)

    # Return both prediction and corrrection. This is for plotting only.
    # Normally, this would just return the correction.
    return (prediction, correction)

#
# Main
#
if __name__ == '__main__':
    arena = (0,200)
    Dist = Distribution.gaussian  # Distribution.triangle or Distribution.gaussian.

    # Start position. Well known, so the distribution is narrow.
    position = Dist(10, 1)

    # Controls and measurements.
    controls = [ Dist(40, 10), Dist(70, 10) ]
    measurements = [ Dist(60, 10), Dist(140, 20) ]

    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         color='#00FFFF', linestyle='steps')

    # This is the filter loop.
    for i in xrange(len(controls)):
        # Call the filter step. The corrected distribution becomes the new position.
        (prediction, position) = histogram_filter_step(position, controls[i], measurements[i])
        histogram_control_plot(controls[i], prediction, measurements[i], position)

    ylim(0.0, 0.16)
    show()
