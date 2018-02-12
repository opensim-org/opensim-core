# -------------------------------------------------------------------------- #
# OpenSim Muscollo: plot_iterate.py                                          #
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
import math
import os
import datetime

import numpy as np
import pylab as pl

import argparse

parser = argparse.ArgumentParser(
    description="Plot 1 or more MucoIterates on the same axes.")
parser.add_argument('file', type=str, nargs='+',
                    help="Paths to MucoIterate files.")
parser.add_argument('--zero', action='store_true',
                    help="Plot y=0 on all plots.")
parser.add_argument('--common', action='store_true',
                    help="Plot only common data columns across files.")
parser.add_argument('--title', action='store',
                    help="Set the title for the plot window")

args = parser.parse_args()

datafiles = args.file

include_zero = args.zero

common_cols = args.common

window_title = args.title


data = list()
column_names = list()
curr_column_names = list()
for ifile, filepath in enumerate(datafiles):
    num_header_rows = 1
    with open(filepath) as f:
        for line in f:
            if not line.startswith('endheader'):
                num_header_rows += 1
            else:
                break
    this_data = np.genfromtxt(filepath, names=True, delimiter='\t',
                              skip_header=num_header_rows)
    data.append(this_data)

    if common_cols:
        # Skip time column.
        for name in this_data.dtype.names[1:]:
            curr_column_names.append(name)

        # Unless first file in datafiles, find intersection of column names
        if ifile == 0:
            column_names = curr_column_names
        else:
            column_names = list(set(column_names) & set(curr_column_names))

        # Without this, the ordering of plots is not deterministic.
        column_names.sort()

        # Clear current column names list.
        curr_column_names = []
    else:
        # Skip time column.
        for name in this_data.dtype.names[1:]:
            if (not name in column_names):
                column_names.append(name)

# If headers have a common prefix, remove it (to avoid very long plot titles).
name_prefix = os.path.commonprefix(column_names)
plot_names = list()
for i, name in enumerate(column_names):
    plot_names.append(column_names[i][len(name_prefix):])
num_plots = len(plot_names)

# Legend entries are the unique parts of the data file names.
legend_nprefix = len(os.path.commonprefix(datafiles))
# Get common suffix by reversing the strings.
legend_nsuffix = len(os.path.commonprefix([s[::-1] for s in datafiles]))
legend_entries = [df[legend_nprefix:(len(df)-legend_nsuffix)]
                  for df in datafiles]

if num_plots < 5:
    num_rows = num_plots
    num_cols = 1
else:
    aspect_ratio = 0.50 # width of figure is 0.75 * height of figure.
    num_rows = math.ceil(math.sqrt(float(num_plots) / aspect_ratio))
    num_cols = math.ceil(aspect_ratio * num_rows)
    if (num_rows * num_cols) > (num_plots + num_cols):
        # There's an extra row that we don't need.
        num_rows -= 1

now = datetime.datetime.now()
if not window_title: 
    window_title = args.file[0]
    if len(args.file) > 1: window_title += ' and others'
    window_title += ' (%s)' % now.strftime('%Y-%m-%dT%H:%M')
fig = pl.figure(figsize=(4 * num_cols, 2 * num_rows), num=window_title)

for i in range(num_plots):
    ax = fig.add_subplot(num_rows, num_cols, i + 1)
    if include_zero:
        ax.axhline(0, color='gray', alpha=0.5)
    if i == num_plots - 1:
        ax.set_xlabel('time')
    ax.set_title(plot_names[i])
    for idat, dat in enumerate(data):
        if column_names[i] in dat.dtype.names:
            ax.plot(dat['time'], dat[column_names[i]])
        else:
            # If not plotting only common data commons, plot something so that 
            # the datafile colors are consistent across plots, but have
            # nothing nothing show up on the graph.
            if not common_cols:
                ax.plot(np.nan, np.nan)
    if i == 0:
        ax.legend(legend_entries)

#fig.subplots_adjust()
fig.tight_layout()
pl.show()
