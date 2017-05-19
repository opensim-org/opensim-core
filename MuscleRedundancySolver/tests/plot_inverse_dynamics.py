import sys

import pylab as pl
import pandas as pd

if len(sys.argv) != 2:
    raise Exception("Requires actual inverse dynamics csv file as argument.")

filtered = pd.read_csv('DEBUG_desiredMoments.csv', index_col=0, header=None)

actual = pd.read_csv(sys.argv[1], index_col=0, skiprows=3)

fig = pl.figure()
num_columns = len(filtered.columns)
for i in range(num_columns):

    ax = fig.add_subplot(num_columns, 1, i + 1)
    ax.plot(filtered.index, filtered[filtered.columns[i]], label='filtered')
    ax.plot(actual.index, actual[actual.columns[i]], label='actual')
    pl.legend()

pl.show()