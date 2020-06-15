# -------------------------------------------------------------------------- #
# OpenSim Moco: plot_gait10dof18musc_INDYGO.py                               #
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
import pylab as pl
import pandas as pd

suffixes = [
        'activation',
        'excitation',
        'norm_fiber_length',
        'norm_fiber_velocity',
        #'norm_tendon_force',
        'other_controls',
        'tendon_force'
        ]

pl.ion()

for suffix in suffixes:
    df = pd.read_csv('testGait10dof18musc_INDYGO_solution_%s.sto' % suffix,
            skiprows=3, index_col=0, delimiter='\t')
    std = pd.read_csv('std_testGait10dof18musc_INDYGO_solution_%s.sto' % suffix,
            skiprows=3, index_col=0, delimiter='\t')
    fig =pl.figure(figsize=(4 * len(std.columns), 4))
    fig.suptitle(suffix)
    for i, col in enumerate(std.columns):
        ax = fig.add_subplot(1, len(std.columns), i + 1)
        pl.plot(std.index, std[col], label='std')
        pl.plot(df.index, df[col], label='actual')
        if i == 0:
            pl.legend()
#    df.plot(subplots=True)
#    std.plot(subplots=True)

df_lm = pd.read_csv('testGait10dof18musc_INDYGO_solution_norm_fiber_length.sto',
        skiprows=3, index_col=0, delimiter='\t')
df_vm = pd.read_csv('testGait10dof18musc_INDYGO_solution_norm_fiber_velocity.sto',
        skiprows=3, index_col=0, delimiter='\t')
import numpy as np
fig =pl.figure(figsize=(4 * len(df_lm.columns), 4))
for i, col in enumerate(df_lm.columns):
    ax = fig.add_subplot(1, len(df_lm.columns), i + 1)
    pl.plot(df_lm.index[:-1],
            np.diff(df_lm[col]) / np.diff(df_lm.index),
            label='finite diff')
    pl.plot(df_vm.index, 10.0 * df_vm[col], label='post-hoc')
    #pl.plot(df_vm.index, 2.0 * 10.0 * df_vm[col], label='post-hoc x 2')
    ax.set_title(col)
    if i == 0:
        pl.legend()


