# -------------------------------------------------------------------------- #
# OpenSim Moco: plot_inverse_dynamics.py                                     #
# -------------------------------------------------------------------------- #
# Copyright (c) 2017 Stanford University and the Authors                     #
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
