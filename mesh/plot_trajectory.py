#/usr/bin/python

import sys

import numpy as np
import pylab as pl

if len(sys.argv) != 2:
    raise Exception("Only one argument allowed.")

data_filepath = sys.argv[1]

data = np.genfromtxt(data_filepath, names=True, delimiter=',')
fig = pl.figure()
num_plots = len(data.dtype.names) - 1
for i in range(num_plots):
    ax = fig.add_subplot(num_plots, 1, i + 1)
    name = data.dtype.names[i + 1]
    ax.plot(data['time'], data[name])
    ax.set_title(name)
pl.show()


