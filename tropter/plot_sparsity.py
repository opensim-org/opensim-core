import scipy.sparse
import pandas as pd
import sys
import numpy as np
import pylab as pl

df = pd.read_csv(sys.argv[1])

spmat = scipy.sparse.coo_matrix((np.ones_like(df.index),
        (df['row_indices'], df['column_indices'])))
pl.spy(spmat)
pl.show()
