#!/usr/bin/env python3
# -------------------------------------------------------------------------- #
# OpenSim Moco: generate_report.py                                           #
# -------------------------------------------------------------------------- #
# Copyright (c) 2019 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Nicholas Bianco                                                 #
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
import ntpath
import math
import opensim as osim
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.ticker import FormatStrFormatter
import matplotlib.cm as cm
import numpy as np
from collections import defaultdict, OrderedDict
# import pdb



## Helper functions.
# ==================
# Convert a SimTK::Vector to a NumPy array for plotting.
# TODO: put something similar in the bindings.
def convert(simtkVector):
    n = simtkVector.size()
    vec = np.empty(n)
    for elt in range(n):
        vec[elt] = simtkVector[elt]

    return vec

# Get a plot label given a OpenSim::Coordinate::MotionType enum and a kinematic
# level ('value' or 'speed')
def getLabelFromMotionType(motionTypeEnum, level):
    label = ''
    if motionTypeEnum == 1:
        if level == 'value': label = 'angle (rad)'
        elif level == 'speed': label = 'ang. vel. (rad/s)'
        elif level == 'accel': label = 'ang. accel. (rad/s^2)'
        else: label = 'rotate'
        return label
    elif motionTypeEnum == 2:
        if level == 'value': label = 'position (m)'
        elif level == 'speed': label = 'velocity (m/s)'
        elif level == 'accel': label = 'accel. (m/s^s)'
        else: label = 'translate'
        return label
    elif motionTypeEnum == 3:
        return 'coupled'
    else:
        return 'undefined'

# Given a state or control name with substring identifying either the left or
# right limb, remove the substring and return the updated name. This function
# also takes the argument 'ls_dict', which is a dictionary of plot linestyles
# corresponding to the right leg (solid line) or left leg (dashed line); it is
# updated here for convenience.
def bilateralize(name, ls_dict):
    if '_r/' in name:
        name = name.replace('_r/', '/')
        ls_dict[name].append('-')
    elif '_l/' in name:
        name = name.replace('_l/', '/')
        ls_dict[name].append('--')
    elif '_r_' in name:
        name = name.replace('_r_', '_')
        ls_dict[name].append('-')
    elif '_l_' in name:
        name = name.replace('_l_', '_')
        ls_dict[name].append('--')
    elif name[-2:] == '_r':
        name = name[:-2]
        ls_dict[name].append('-')
    elif name[-2:] == '_l':
        name = name[:-2]
        ls_dict[name].append('--')
    else:
        ls_dict[name].append('-')

    return name, ls_dict

def getIndexForNearestValue(vec, val):
    return min(range(len(vec)), key=lambda i: abs(vec[i]-val))

def truncate(string, max_length):
    """https://www.xormedia.com/string-truncate-middle-with-ellipsis/"""
    if len(string) <= max_length:
        # string is already short-enough
        return string
    # half of the size, minus the 3 .'s
    n_2 = int(max_length / 2 - 3)
    # whatever's left
    n_1 = max_length - n_2 - 3
    return '{0}...{1}'.format(string[:n_1], string[-n_2:])


class Report(object):
    def __init__(self,
                 model,
                 iterate_filepath,
                 bilateral=True,
                 ref_files=None,
                 colormap=None,
                 ):
        self.model = model
        self.iterate_filepath = iterate_filepath
        self.iterate = osim.MocoIterate(self.iterate_filepath)
        self.bilateral = bilateral
        self.ref_files = ref_files
        self.colormap = colormap

        # Get any reference files provided by the user and create a list of NumPy
        # arrays to use in plotting.
        self.refs = list()
        if ref_files != None:
            for ref_file in ref_files:
                num_header_rows = 1
                with open(ref_file) as f:
                    for line in f:
                        if not line.startswith('endheader'):
                            num_header_rows += 1
                        else:
                            break
                this_ref = np.genfromtxt(ref_file, names=True, delimiter='\t',
                                         skip_header=num_header_rows)
                self.refs.append(this_ref)

        # Load the colormap provided by the user. Use a default colormap ('jet') if
        # not provided. Uniformly sample the colormap based on the number of reference
        # data sets, plus one for the MocoIterate.
        if colormap is None: colormap = 'jet'
        self.cmap_samples = np.linspace(0.1, 0.9, len(self.refs)+1)
        self.cmap = cm.get_cmap(colormap)

        ## Legend handles and labels.
        # ===========================
        # Create legend handles and labels that can be used to create a figure legend
        # that is applicable all figures.
        self.legend_handles = list()
        self.legend_labels = list()
        all_files = list()
        if ref_files != None: all_files += ref_files
        all_files.append(iterate_filepath)
        for sample, file in zip(self.cmap_samples, all_files):
            color = self.cmap(sample)
            if bilateral:
                r = mlines.Line2D([], [], ls='-', color=color, linewidth=3)
                self.legend_handles.append(r)
                self.legend_labels.append(file + ' (right leg)')
                l = mlines.Line2D([], [], ls='--', color=color, linewidth=3)
                self.legend_handles.append(l)
                self.legend_labels.append(file + ' (left leg)')
            else:
                h = mlines.Line2D([], [], ls='-', color=color, linewidth=3)
                self.legend_handles.append(h)
                self.legend_labels.append(file)

        # Time
        # -----
        # Convert iterate time vector to a plotting-friendly NumPy array.
        self.time = convert(self.iterate.getTime())
        # Create a conservative set of x-tick values based on the time vector.
        nexttime = math.ceil(self.time[0] * 10) / 10
        nexttolast = math.floor(self.time[-1] * 10) / 10
        if (nexttolast - nexttime) < 1.5:
            self.timeticks = np.arange(nexttime, nexttolast, 0.2)
        else:
            self.timeticks = None

        self.plots_per_page = 15.0
        self.num_cols = 3
        # Add an extra row to hold the legend and other infromation.
        self.num_rows = (self.plots_per_page / 3) + 1

    def getVariable(self, type, path):
        if type == 'state':
            var = convert(self.iterate.getState(path))
        elif type == 'control':
            var = convert(self.iterate.getControl(path))
        elif type == 'multiplier':
            var = convert(self.iterate.getMultiplier(path))
        elif type == 'derivative':
            derivativesTraj = self.iterate.getDerivativesTrajectory()
            derivativeNames = self.iterate.getDerivativeNames()
            count = 0
            col = 0
            for derivName in derivativeNames:
                if derivName == path:
                    col = count
                count += 1

            n = derivativesTraj.nrow()
            var = np.empty(n)
            for row in range(n):
                var[row] = derivativesTraj.get(row, col)
        elif type == 'slack':
            var = convert(self.iterate.getSlack(path))
        elif type == 'parameter':
            var = convert(self.iterate.getParameter(path))

        return var

    def plotVariables(self, var_type, var_dict, ls_dict, label_dict):
        # Loop through all keys in the dictionary and plot all variables.
        p = 1 # Counter to keep track of number of plots per page.
        for i, key in enumerate(var_dict.keys()):
            # If this is first key or if we filled up the previous page with
            # plots, create a new figure that will become the next page.
            if p % self.plots_per_page == 1:
                fig = plt.figure(figsize=(8.5, 11))

            plt.subplot(self.num_rows, self.num_cols, p + 3)
            # Loop through all the state variable paths for this key.
            ymin = np.inf
            ymax = -np.inf
            for path, ls in zip(var_dict[key], ls_dict[key]):
                var = self.getVariable(var_type, path)
                ymin = np.min(var)
                ymax = np.max(var)
                # If any reference data was provided, that has columns matching
                # the current variable path, then plot them first.
                for r, ref in enumerate(self.refs):
                    # Column names for reference data are read in with no
                    # slashes.
                    pathNoSlashes = path.replace('/', '')
                    if pathNoSlashes in ref.dtype.names:
                        init = getIndexForNearestValue(ref['time'], time[0])
                        final = getIndexForNearestValue(ref['time'], time[-1])
                        y = ref[pathNoSlashes][init:final]
                        plt.plot(ref['time'][init:final],
                                 y, ls=ls,
                                 color=cmap(cmap_samples[r]),
                                 linewidth=2.5)
                        ymin = np.minimum(ymin, np.min(y))
                        ymax = np.maximum(ymax, np.max(y))

                # Plot the variable values from the MocoIterate.
                plt.plot(self.time, var, ls=ls, color=self.cmap(
                    self.cmap_samples[len(self.refs)]),
                         linewidth=1.5)

            # Plot labels and settings.
            plt.title(truncate(key, 38), fontsize=10)
            plt.xlabel('time (s)', fontsize=8)
            plt.ylabel(label_dict[key], fontsize=8)
            if not self.timeticks is None:
                plt.xticks(self.timeticks)
            plt.xticks(fontsize=6)
            plt.yticks(fontsize=6)
            plt.xlim(self.time[0], self.time[-1])
            if 0 <= ymin and ymax <= 1:
                plt.ylim(0, 1)
            plt.ticklabel_format(axis='y', style='sci', scilimits=(-3, 3))
            ax = plt.gca()
            ax.get_yaxis().get_offset_text().set_position((-0.15,0))
            ax.get_yaxis().get_offset_text().set_fontsize(6)
            ax.tick_params(direction='in', gridOn=True)
            ax.xaxis.set_major_formatter(
                FormatStrFormatter('%.1f'))

            # If we filled up the current figure or ran out of keys, add this
            # figure as a new page to the PDF. Otherwise, increment the plot
            # counter and keep going.
            if (p % self.plots_per_page == 0) or (i == len(var_dict.keys())-1):
                fig.tight_layout()
                plt.figlegend(self.legend_handles, self.legend_labels,
                              loc='upper center', bbox_to_anchor=(0.5, 0.97),
                              fancybox=True, shadow=True)
                self.pdf.savefig(fig)
                plt.close()
                p = 1
            else:
                p += 1

    def generate(self):

        ## Generate report.
        # =================

        # TODO is ntpath cross-platform?
        iterate_fname = ntpath.basename(self.iterate_filepath)
        iterate_fname = iterate_fname.replace('.sto', '')
        iterate_fname = iterate_fname.replace('.mot', '')
        with PdfPages(iterate_fname + '_report.pdf') as self.pdf:

            # States & Derivatives
            # --------------------
            state_names = self.iterate.getStateNames()
            derivative_names = self.iterate.getDerivativeNames()
            derivs = True if (len(derivative_names) > 0) else False
            if len(state_names) > 0:
                # Loop through the model's joints and cooresponding coordinates to
                # store plotting information.
                state_dict = OrderedDict()
                state_ls_dict = defaultdict(list)
                state_label_dict = dict()
                derivative_dict = OrderedDict()
                derivative_ls_dict = defaultdict(list)
                derivative_label_dict = dict()
                coordSet = model.getCoordinateSet()
                for c in range(coordSet.getSize()):
                    coord = coordSet.get(c)
                    coordName = coord.getName()
                    coordPath = coord.getAbsolutePathString()
                    coordMotType = coord.getMotionType()
                    # Append suffixes to create names for position and speed state
                    # variables.
                    valueName = coordName + '/value'
                    speedName = coordName + '/speed'
                    if derivs: accelName = coordName + '/accel'
                    if self.bilateral:
                        # If the --bilateral flag was set by the user, remove
                        # substrings that indicate the body side and update the
                        # linestyle dict.
                        valueName, state_ls_dict = bilateralize(valueName,
                                state_ls_dict)
                        speedName, state_ls_dict = bilateralize(speedName,
                                state_ls_dict)
                        if derivs:
                            accelName, derivative_ls_dict = bilateralize(accelName,
                                    derivative_ls_dict)

                    else:
                        state_ls_dict[valueName].append('-')
                        state_ls_dict[speedName].append('-')
                        if derivs: derivative_ls_dict[accelName].append('-')


                    if not valueName in state_dict:
                        state_dict[valueName] = list()
                    # If --bilateral was set, the 'valueName' key will correspond
                    # to a list containing paths for both sides of the model.
                    state_dict[valueName].append(coordPath + '/value')
                    state_label_dict[valueName] = \
                        getLabelFromMotionType(coordMotType, 'value')

                    if not speedName in state_dict:
                        state_dict[speedName] = list()
                    state_dict[speedName].append(coordPath + '/speed')
                    state_label_dict[speedName] = \
                        getLabelFromMotionType(coordMotType, 'speed')

                    if derivs:
                        if not accelName in derivative_dict:
                            derivative_dict[accelName] = list()
                        derivative_dict[accelName].append(coordPath + '/accel')
                        derivative_label_dict[accelName] = \
                            getLabelFromMotionType(coordMotType, 'accel')

                self.plotVariables('state', state_dict, state_ls_dict, state_label_dict)
                if derivs:
                    self.plotVariables('derivative', derivative_dict, derivative_ls_dict,
                            derivative_label_dict)

            # Controls
            # --------
            control_names = self.iterate.getControlNames()
            if len(control_names) > 0:
                control_dict = OrderedDict()
                ls_dict = defaultdict(list)
                label_dict = dict()
                for control_name in control_names:
                    title = control_name.replace('/', '')
                    if self.bilateral:
                        # If the --bilateral flag was set by the user, remove
                        # substrings that indicate the body side and update the
                        # linestyle dict.
                        title, ls_dict = bilateralize(title, ls_dict)
                    else:
                        ls_dict[title].append('-')

                    if not title in control_dict:
                        control_dict[title] = list()
                    # If --bilateral was set, the 'title' key will correspond
                    # to a list containing paths for both sides of the model.
                    control_dict[title].append(control_name)
                    label_dict[title] = ''
                self.plotVariables('control', control_dict, ls_dict, label_dict)

            # Multipliers
            # -----------
            multiplier_names = self.iterate.getMultiplierNames()
            if len(multiplier_names) > 0:
                multiplier_dict = OrderedDict()
                ls_dict = defaultdict(list)
                label_dict = dict()
                for multiplier_name in multiplier_names:
                    if self.bilateral:
                        # If the --bilateral flag was set by the user, remove
                        # substrings that indicate the body side and update the
                        # linestyle dict.
                        title, ls_dict = bilateralize(multiplier_name, ls_dict)
                    else:
                        ls_dict[valueName].append('-')

                    if not title in multiplier_dict:
                        multiplier_dict[title] = list()
                    # If --bilateral was set, the 'title' key will correspond
                    # to a list containing paths for both sides of the model.
                    multiplier_dict[title].append(multiplier_name)
                    label_dict[title] = ''

                self.plotVariables('multiplier', multiplier_dict, ls_dict, label_dict)

            # Parameters
            # ----------
            # TODO: this is a crude first attempt, need to refine.
            parameter_names = self.iterate.getParameterNames()
            if len(parameter_names) > 0:
                fig = plt.figure(figsize=(8.5, 11))
                fig.patch.set_visible(False)
                ax = plt.axes()

                cell_text = []
                parameters = convert(iterate.getParameters())
                cell_text.append(['%10.5f' % p for p in parameters])

                print('iterate name', iterate_fname)
                plt.table(cellText=cell_text, rowLabels=parameter_names,
                          colLabels=[iterate_fname], loc='center')
                ax.axis('off')
                ax.axis('tight')

                plt.subplots_adjust(left=0.2, right=0.8)

                plt.figlegend(legend_handles, legend_labels,
                    loc='upper center', bbox_to_anchor=(0.5, 0.97),
                    fancybox=True, shadow=True)
                pdf.savefig(fig)
                plt.close()

            # Slacks
            # ------
            # TODO slacks not accessible through MocoIterate
            # TODO should we even plot these?


if __name__ == "__main__":
    import argparse

    ## Input parsing.
    ## =============
    parser = argparse.ArgumentParser(
        description="Generate a report given a MocoIterate and an associated "
                    "OpenSim Model. Optionally, additional reference data "
                    "compatible with the MocoIterate may be plotted "
                    "simultaneously.")
    # Required arguments.
    parser.add_argument('model', type=str,
                        help="OpenSim Model file name (including path).")
    parser.add_argument('iterate', type=str,
                        help="MocoIterate file name (including path).")
    # Optional arguments.
    parser.add_argument('--bilateral', action='store_true',
                        help="Plot left and right limb states and controls "
                             "together.")
    parser.add_argument('--refs', type=str, nargs='+',
                        help="Paths to reference data files.")
    parser.add_argument('--colormap', type=str,
                        help="Matplotlib colormap from which plot colors are "
                             "sampled from.")
    args = parser.parse_args()

    # Load the Model and MocoIterate from file.
    model = osim.Model(args.model)

    ref_files = args.refs

    report = Report(model=model,
                    iterate_filepath=args.iterate,
                    bilateral=args.bilateral,
                    colormap=args.colormap,
                    ref_files=ref_files,
                    )
    report.generate()

