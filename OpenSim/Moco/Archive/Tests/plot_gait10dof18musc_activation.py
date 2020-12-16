# -------------------------------------------------------------------------- #
# OpenSim Moco: plot_gait10dof18musc_activation.py                           #
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
import math

cycle_start = 0.58
cycle_end = 1.81
duration_of_gait_cycle = cycle_end - cycle_start
half_gait_cycle = 0.5 * duration_of_gait_cycle 

muscles = ['glut_max', 'iliopsoas', 'rect_fem',
           'hamstrings', 'bifemsh', 'vasti',
           'gastroc', 'soleus', 'tib_ant']

sol = pd.read_csv('GlobalStaticOptimization_OCP_solution.csv', index_col=0,
        skiprows=2)
#sol = pd.read_csv('INDYGO_OCP_solution.csv', index_col=0,
#        skiprows=2)
# sol.plot()
#num_muscles = 0
#plot_names = list()
#for col in sol.columns:
#    if col.endswith('activation'):
#        num_muscles += 1
#        plot_names.append(col)
col_indices_r = list()
col_indices_l = list()
for muscle in muscles:
    for i, col in enumerate(sol.columns):
        if muscle + '_r' in col and 'activation' in col:
            col_indices_r.append(i)
        if muscle + '_l' in col and 'activation' in col:
            col_indices_l.append(i)



num_cols = 3
num_rows = 3 #math.ceil(float(num_muscles) / num_cols)
pl.figure(figsize=(4 * num_cols, 3 * num_rows))
pgc_r = 100.0 * (sol.index - cycle_start) / duration_of_gait_cycle
for i in range(len(muscles)):
    ax = pl.subplot(num_rows, num_cols, i + 1)
    col_label_r = sol.columns[col_indices_r[i]]
    ax.plot(pgc_r, sol[col_label_r])
    #col_label_l = sol.columns[col_indices_l[i]]
    #ax.plot(sol.index + half_gait_cycle, sol[col_label_l])
    ax.set_title(col_label_r.split('/')[-1].replace('_r_activation', ''))
    ax.set_ylim(0, 1)
    if i == 3:
        ax.set_ylabel('activation')
    if i < 6:
        ax.set_xticklabels([])
    i_col = i % num_cols
    if i_col > 0:
        ax.set_yticklabels([])
    if i == 7:
        ax.set_xlabel('time (% gait cycle)')

pl.savefig('gait10dof18musc_activation.png')
