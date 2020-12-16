# -------------------------------------------------------------------------- #
# OpenSim Muscollo: plot_casadi_sparsity.py                                  #
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

# This command-line utility plots the sparsity pattern from a file. To use this
# script, first generate a sparsity pattern file using MocoCasADiSolver's
# optim_write_sparsity property.
#
# Usage:
#       python plot_casadi_sparsity.py <prefix>_constraint_Jacobian_sparsity.mtx
#
# In this example, <prefix> is the value of the optim_write_sparsity property.

df = pd.read_csv(sys.argv[1], skiprows=2, sep=' ',
                 names=['row_indices', 'column_indices'])

with open(sys.argv[1]) as f:
    # The first line is a comment.
    f.readline()
    # The second line contains the number of row, columns, and nonzeroes.
    line1 = f.readline()
    numbers = line1.split(' ')
    num_rows = int(numbers[0])
    num_cols = int(numbers[1])

spmat = scipy.sparse.coo_matrix((np.ones_like(df.index),
        (df['row_indices'] - 1, df['column_indices'] - 1)),
        shape=(num_rows, num_cols))
pl.spy(spmat, markersize=2, markeredgecolor='k', marker='.')
pl.show()
