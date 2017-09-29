# Discrete distribution, that holds 'values' for the indices
# 'start', 'start'+1, ... 'start'+len(values)-1.
# The values are non-negative and sum up to 1.0.
# For example,
# start = 3, values = [0.25 0.5 0.25] represents the distribution:
# ... 0.0   0.0  0.0  0.0  0.0  0.25 0.5  0.25 0.0  0.0  0.0 ...
# ... -2    -1    0    1    2    3    4    5    6    7    8  ...
# and start = 2, values = [0.0 0.25 0.5 0.25] represents the same
# distribution.
# Claus Brenner, 26 OCT 2012
from math import exp, ceil

class Distribution:
    """This class represents a discrete distribution."""
    def __init__(self, offset = 0, values = [1.0]):
        self.offset = offset
        self.values = values[:]

    def __repr__(self):
        s = "start = %d, values =" % self.offset
        for x in self.values:
            s += " %f" % x
        return s

    def start(self):
        return self.offset

    def stop(self):
        """Return the stop point of the distribution, which is the first index
           'outside' the distribution."""
        return self.offset + len(self.values)

    def normalize(self):
        """Normalizes a distribution so that the sum of all values is 1.0."""
        s = float(sum(self.values))
        if s != 0.0:
            self.values = [i / s for i in self.values]

    def value(self, index):
        index -= self.offset
        if index < 0 or index >= len(self.values):
            return 0.0
        else:
            return self.values[index]

    def plotlists(self, start = None, stop = None):
        if start == None:
            start = self.start()
        if stop == None:
            stop = self.stop()
        if start <= stop:
            indices = [i + 0.5 for i in xrange(start, stop)]
            vals = [self.value(i) for i in xrange(start, stop)]
            return (indices, vals)
        else:
            return ([], [])

    @staticmethod
    def unit_pulse(center):
        """Returns a unit pulse at center."""
        return Distribution(center, [1.0])

    @staticmethod
    def triangle(center, half_width):
        """Returns a triangular distribution. The peak is at 'center' and it is
           zero at center +/- half_width. center and half_width are integers."""
        w = int(half_width)
        c = int(center)
        values = []
        for i in xrange(-w+1, 0):
            values.append(w+i)
        for i in xrange(0, w):
            values.append(w-i)
        d = Distribution(center-w+1, values)
        d.normalize()
        return d

    @staticmethod
    def gaussian(mu, sigma, cut = 5.0):
        """Returns a gaussian distribution, centered at mu, with variance
           sigma**2. For efficiency reasons, the tails are cut at
           cut * sigma, so with cut=5, it will fill the array from -5 sigma
           to +5 sigma."""
        sigma2 = sigma * sigma
        extent = int(ceil(cut * sigma))
        values = []
        for x in xrange(mu - extent, mu + extent + 1):
            values.append(exp((-0.5*(x-mu)*(x-mu))/sigma2))
        d = Distribution(mu - extent, values)
        d.normalize()
        return d

    @staticmethod
    def sum(distributions, weights = None):
        """Returns the sum of all distributions (which is a list of Distribution
           objects). If weights (a list) is specified, it must specify one float
           value for each distribution."""
        # If weights are not given, generate them, all 1.0's.
        if not weights:
            weights = [1.0 for d in distributions]
        # First make an all-zero list which covers all indices.
        start = min([d.start() for d in distributions])
        stop  = max([d.stop() for d in distributions])
        sum_dist = [0.0 for _ in xrange(start, stop)]
        for i in xrange(len(distributions)):
            dist = distributions[i]
            # Now weight all values and add them to sum_dist.
            for j in xrange(len(dist.values)):
                sum_dist[dist.start()-start+j] += dist.values[j] * weights[i]
        d = Distribution(start, sum_dist)
        Distribution.normalize(d)
        return d
