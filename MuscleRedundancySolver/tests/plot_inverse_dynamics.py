import pylab as pl
import pandas as pd

filtered = pd.read_csv('DEBUG_desiredMoments.csv', index_col=0, header=None)

actual = pd.read_csv('DEBUG_testTugOfWar_MRS_actualInvDyn.csv', index_col=0,
                     skiprows=3)

fig = pl.figure()
ax = fig.add_subplot(1, 1, 1)
ax.plot(filtered.index, filtered[filtered.columns[0]], label='filtered')
ax.plot(actual.index, actual[actual.columns[0]], label='actual')

pl.show()