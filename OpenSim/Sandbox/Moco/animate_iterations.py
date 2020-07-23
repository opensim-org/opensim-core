#!/usr/bin/env python3
# -------------------------------------------------------------------------- #
# OpenSim Moco: animate_iterations.py                                        #
# -------------------------------------------------------------------------- #
# Copyright (c) 2019 Stanford University and the Authors                     #
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
import os
import numpy as np
import opensim as osim
import pandas as pd
import matplotlib
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import pylab as pl
from matplotlib.animation import FuncAnimation
import tempfile

def animate_iterations(modelIn, traj_prefix):
    xmin = np.inf
    ymin = np.inf
    zmin = np.inf
    xmax = -np.inf
    ymax = -np.inf
    zmax = -np.inf

    data = list()
    dir, prefix = os.path.split(traj_prefix)
    files = sorted(os.listdir(dir))
    for file in files:
        if file.startswith(prefix):
            model = osim.Model(modelIn)
            trajectory = osim.MocoTrajectory(os.path.join(dir, file))
            statesTab = trajectory.exportToStatesTable()
            osim.STOFileAdapter.write(statesTab, 'statestabTODO.sto')
            states_traj = osim.StatesTrajectory.createFromStatesStorage(model,
                                                                        'statestabTODO.sto')

            reporter = osim.TableReporterVec3()
            model.finalizeFromProperties()
            for comp in model.getComponentsList():
                marker = osim.Marker.safeDownCast(comp)
                if marker:
                    reporter.addToReport(marker.getOutput('location'))
            model.addComponent(reporter)
            model.initSystem()
            for state in states_traj:
                model.realizeReport(state)
            table = reporter.getTable().flatten(['_x', '_y', '_z'])
            filepath = "TODO.sto"
            osim.STOFileAdapterVec3.write(reporter.getTable(), 'TODO3.sto')
            osim.STOFileAdapter.write(table, filepath)

            num_header_rows = 1
            with open(filepath) as f:
                for line in f:
                    if not line.startswith('endheader'):
                        num_header_rows += 1
                    else:
                        break
            this_data = pd.read_csv(filepath, delimiter='\t', index_col=0,
                                    header=num_header_rows)
            assert (len(this_data.columns) % 3 == 0)
            xmin = min(xmin, np.min(this_data.iloc[:, 0::3].to_numpy(),
                                    keepdims=False))
            xmax = max(xmax, np.max(this_data.iloc[:, 0::3].to_numpy(),
                                    keepdims=False))
            ymin = min(ymin, np.min(this_data.iloc[:, 1::3].to_numpy(),
                                    keepdims=False))
            ymax = max(ymax, np.max(this_data.iloc[:, 1::3].to_numpy(),
                                    keepdims=False))
            zmin = min(zmin, np.min(this_data.iloc[:, 2::3].to_numpy(),
                                    keepdims=False))
            zmax = max(zmax, np.max(this_data.iloc[:, 2::3].to_numpy(),
                                    keepdims=False))
            data.append(this_data)

    assert (len(data) > 0)
    # TODO uniform time sampling for interpretable results!!!
    fig = pl.figure()
    ax = fig.add_subplot(111, projection='3d')
    # graph, = ax.plot([], [], [], color='k',
    #                  linestyle=' ',
    #                  marker='o')
    len_data = float(len(data))
    def init():
        ax.set_axis_off()
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(zmin, zmax)
        ax.set_zlim(ymin, ymax)

    def update(frame):
        # graph.set_data(data[frame].iloc[:, 0::3].values.flatten(),
        #                data[frame].iloc[:, 1::3].values.flatten())
        # graph.set_3d_properties(data[frame].iloc[:, 2::3].values.flatten())
        pl.cla()
        ax.set_axis_off()
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(zmin, zmax)
        ax.set_zlim(ymin, ymax)
        for i in range(frame - 1, frame):
            for j in range(int(len(data[i].columns) / 3)):
                ax.plot(data[i].iloc[:, 3 * j].values.flatten(),
                        data[i].iloc[:, 3 * j + 2].values.flatten(),
                        data[i].iloc[:, 3 * j + 1].values.flatten(),
                        #color=(1 - i / float(frame),
                        #       1 - i / float(frame),
                        #       1 - i / float(frame)),
                        linestyle='-',
                        )
            # ax.plot(data[i].iloc[:, 0::3].values.flatten(),
            #         data[i].iloc[:, 2::3].values.flatten(),
            #         data[i].iloc[:, 1::3].values.flatten(),
            #         color=(1 - i / float(frame),
            #                1 - i / float(frame),
            #                1 - i / float(frame)),
            #         linestyle=' ',
            #         marker='o',
            #         markersize=1,
            #         )
        ax.set_title(f'iteration {frame}')
        # return graph,

    animation = FuncAnimation(fig, update,
                              init_func=init,
                              frames=range(len(data)))

    pl.show()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="TODO")
    # Required arguments.
    parser.add_argument('model-or-study', type=str,
                        help="OpenSim Model file name (including path).")
    parser.add_argument('trajectory-prefix', type=str,
                        help="TODO")
    args = parser.parse_args()
    args = vars(args)

    if args['model-or-study'].endswith('.osim'):
        model = osim.Model(args['model-or-study'])
    elif args['model-or-study'].endswith('.omoco'):
        study = osim.MocoStudy(args['model-or-study'])
        model = study.getProblem().getPhase().getModelProcessor().process()
    else:
        raise Exception('TODO')
    model.finalizeConnections()
    animate_iterations(model, args['trajectory-prefix'])
