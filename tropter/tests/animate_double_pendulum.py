#/usr/bin/python
# -----------------------------------------------------------------------------
# tropter: animate_double_pendulum.py
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

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

if len(sys.argv) != 2:
    raise Exception("Only one argument allowed.")

data_filepath = sys.argv[1]

data = np.genfromtxt(data_filepath, names=True, delimiter=',', skip_header=2)

fig, ax = plt.subplots()

# time = data['time']
q0 = data['q0']
q1 = data['q1']
x0 = np.cos(q0)
y0 = np.sin(q0)
x1 = x0 + np.cos(q0 + q1)
y1 = y0 + np.sin(q0 + q1)
link0, = ax.plot([0, 0], [x0[0], y0[0]])
link1, = ax.plot([x0[0], y0[0]], [x1[0], y1[0]])

ax.set_ylim(-2.5, 2.5)
ax.set_xlim(-2.5, 2.5)

text = plt.text(0, 0, '0')

def animate(i):
    link0.set_xdata([0, x0[i]])
    link0.set_ydata([0, y0[i]])
    link1.set_xdata([x0[i], x1[i]])
    link1.set_ydata([y0[i], y1[i]])
    text.set_text('%i' % i)
    return link0, link1,


# Init only required for blitting to give a clean slate.
def init():
    return link0, link1,

ani = animation.FuncAnimation(fig, animate, np.arange(1, data.shape[0]),
        init_func=init, interval=100, repeat=False)

plt.show()
