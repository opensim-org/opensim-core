# -------------------------------------------------------------------------- #
# OpenSim Muscollo: plot_casadi_iterate.py                                   #
# -------------------------------------------------------------------------- #
# Copyright (c) 2018 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Christopher Dembia                                              #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License"); you may    #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #
import scipy.sparse
import pandas as pd
import sys
import numpy as np
import pylab as pl

"""TODO: Does not work yet.
"""

df = pd.read_csv(sys.argv[1], skiprows=2, sep=None)

with open(sys.argv[1]) as f:
    line0 = f.readline()
    num_rows = int(line0.split('=')[1])
    line1 = f.readline()
    num_cols = int(line1.split('=')[1])

spmat = scipy.sparse.coo_matrix((np.ones_like(df.index),
        (df['row_indices'] - 1, df['column_indices'] - 1)),
        shape=(num_rows, num_cols))
pl.spy(spmat, markersize=2, markeredgecolor='k', marker='.')
pl.show()
