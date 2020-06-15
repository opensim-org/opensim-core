#/usr/bin/python
# -----------------------------------------------------------------------------
# tropter: plot_trajectory.py
# -----------------------------------------------------------------------------
# Copyright (c) 2017 tropter authors
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may
# not use this file except in compliance with the License. You may obtain a
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -----------------------------------------------------------------------------

import sys
import math
import os

import numpy as np
import pylab as pl

if len(sys.argv) == 1 or len(sys.argv) > 3:
    raise Exception("Only 1 or 2 argument allowed.")

data_filepath = sys.argv[1]
if len(sys.argv) == 3:
    include_zero = False
else:
    include_zero = True


data = np.genfromtxt(data_filepath, names=True, delimiter=',', skip_header=3)

names = data.dtype.names[1:]

# If headers have a common prefix, remove it (to avoid very long plot titles).
name_prefix = os.path.commonprefix(names)
plot_names = list()
for i, name in enumerate(names):
    plot_names.append(names[i][len(name_prefix):])
num_plots = len(plot_names)


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
fig = pl.figure(figsize=(4 * num_cols, 2 * num_rows))
for i in range(num_plots):
    ax = fig.add_subplot(num_rows, num_cols, i + 1)
    name = data.dtype.names[i + 1]
    if include_zero:
        ax.axhline(0, color='gray', alpha=0.5)
    ax.plot(data['time'], data[name])
    plot_name = plot_names[i]
    ax.set_title(plot_name)
    if i == num_plots - 1:
        ax.set_xlabel('time')

#fig.subplots_adjust()
fig.tight_layout()
pl.show()
