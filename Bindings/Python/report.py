#!/usr/bin/env python3
# -------------------------------------------------------------------------- #
# OpenSim Moco: report.py                                                    #
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
import numpy as np
from collections import defaultdict, OrderedDict

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
        elif level == 'speed': label = 'speed (rad/s)'
        elif level == 'accel': label = 'accel. (rad/s^2)'
        else: label = 'rotate'
        return label
    elif motionTypeEnum == 2:
        if level == 'value': label = 'position (m)'
        elif level == 'speed': label = 'speed (m/s)'
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
    # Keep modifying the name until no side tags remain.
    isRightLeg = True
    while True:
        if '_r/' in name:
            name = name.replace('_r/', '/')
        elif '_l/' in name:
            name = name.replace('_l/', '/')
            isRightLeg = False
        elif '_r_' in name:
            name = name.replace('_r_', '_')
        elif '_l_' in name:
            name = name.replace('_l_', '_')
            isRightLeg = False
        elif name[-2:] == '_r':
            name = name[:-2]
        elif name[-2:] == '_l':
            name = name[:-2]
            isRightLeg = False
        else:
            if isRightLeg:
                ls_dict[name].append('-')
            else:
                ls_dict[name].append('--')

            break

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
                 trajectory_filepath,
                 bilateral=True,
                 ref_files=None,
                 colors=None,
                 output=None,
                 ):
        self.model = model
        self.model.initSystem()
        self.trajectory_filepath = trajectory_filepath
        self.trajectory = osim.MocoTrajectory(self.trajectory_filepath)
        self.bilateral = bilateral
        self.ref_files = ref_files
        self.colors = colors
        trajectory_fname = ntpath.basename(self.trajectory_filepath)
        trajectory_fname = trajectory_fname.replace('.sto', '')
        trajectory_fname = trajectory_fname.replace('.mot', '')
        self.trajectory_fname = trajectory_fname
        if output:
            self.output = output
        else:
            self.output = trajectory_fname + '_report.pdf'

        # Get any reference files provided by the user and create a list of NumPy
        # arrays to use in plotting.
        self.refs = list()
        if ref_files != None:
            for ref_file in ref_files:
                filename, file_ext = os.path.splitext(ref_file)

                # If the reference is a file with metadata indicating that
                # rotational data is in degrees, convert to radians and write to
                # a new file to be used for plotting.
                # TODO: don't write the extra file permanently
                if file_ext == '.sto' or file_ext == '.mot':
                    ref = osim.TimeSeriesTable(ref_file)
                    if (ref.hasTableMetaDataKey('inDegrees') and 
                            ref.getTableMetaDataAsString('inDegrees') == 'yes'):
                        simbodyEngine = self.model.getSimbodyEngine()
                        simbodyEngine.convertDegreesToRadians(ref)
                        ref_file = ref_file.replace(file_ext, 
                                '_radians' + file_ext)
                        sto_adapter = osim.STOFileAdapter()
                        sto_adapter.write(ref, ref_file)

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

        if self.colors is None:
            self.colors = list()
            import matplotlib
            cmap_samples = np.linspace(0.1, 0.9, len(self.refs)+1)
            cmap = matplotlib.colormaps['jet']
            for sample in cmap_samples:
                self.colors.append(cmap(sample))

        # Check that the colors list is the correct length
        if len(self.colors) != len(self.refs)+1:
            raise Exception(f"The list argument 'colors' should have length equal "
                            f"to the number of files provided (trajectory and "
                            f"reference files), but has length "
                            f"{len(self.colors)}.")

        ## Legend handles and labels.
        # ===========================
        # Create legend handles and labels that can be used to create a figure 
        # legend that is applicable all figures.
        self.legend_handles = list()
        self.legend_labels = list()
        all_files = list()
        if ref_files != None: all_files += ref_files
        all_files.append(trajectory_filepath)
        lw = 8 / len(self.colors)
        if lw < 0.5: lw = 0.5
        if lw > 2: lw = 2
        for color, file in zip(self.colors, all_files):
            import matplotlib.lines as mlines
            if bilateral:
                r = mlines.Line2D([], [], ls='-', color=color, linewidth=lw)
                self.legend_handles.append(r)
                self.legend_labels.append(file + ' (right leg)')
                l = mlines.Line2D([], [], ls='--', color=color, linewidth=lw)
                self.legend_handles.append(l)
                self.legend_labels.append(file + ' (left leg)')
            else:
                h = mlines.Line2D([], [], ls='-', color=color, linewidth=lw)
                self.legend_handles.append(h)
                self.legend_labels.append(file)

        # Time
        # -----
        # Convert trajectory time vector to a plotting-friendly NumPy array.
        self.time = convert(self.trajectory.getTime())
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
        self.num_rows = (self.plots_per_page / self.num_cols) + 1

    def getVariable(self, type, path):
        if type == 'state':
            var = convert(self.trajectory.getState(path))
        elif type == 'input_control':
            var = convert(self.trajectory.getInputControl(path))
        elif type == 'control':
            var = convert(self.trajectory.getControl(path))
        elif type == 'multiplier':
            var = convert(self.trajectory.getMultiplier(path))
        elif type == 'derivative':
            derivativesTraj = self.trajectory.getDerivativesTrajectory()
            derivativeNames = self.trajectory.getDerivativeNames()
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
            var = convert(self.trajectory.getSlack(path))
        elif type == 'parameter':
            var = convert(self.trajectory.getParameter(path))

        return var

    def plotVariables(self, var_type, var_dict, ls_dict, label_dict):
        import matplotlib.pyplot as plt

        # Loop through all keys in the dictionary and plot all variables.
        p = 1 # Counter to keep track of number of plots per page.
        for i, key in enumerate(var_dict.keys()):
            # If this is first key or if we filled up the previous page with
            # plots, create a new figure that will become the next page.
            if p % self.plots_per_page == 1:
                fig = plt.figure(figsize=(8.5, 11))

            plt.subplot(int(self.num_rows), int(self.num_cols),
                        int(p + self.num_cols))
            # Loop through all the state variable paths for this key.
            ymin = np.inf
            ymax = -np.inf
            for path, ls in zip(var_dict[key], ls_dict[key]):
                var = self.getVariable(var_type, path)
                ymin = np.minimum(ymin, np.min(var))
                ymax = np.maximum(ymax, np.max(var))
                # If any reference data was provided, that has columns matching
                # the current variable path, then plot them first.
                for r, ref in enumerate(self.refs):
                    # Column names for reference data are read in with no
                    # slashes.
                    pathNoSlashes = path.replace('/', '')
                    if pathNoSlashes in ref.dtype.names:
                        init = getIndexForNearestValue(ref['time'], self.time[0])
                        final = getIndexForNearestValue(ref['time'], 
                            self.time[-1])
                        y = ref[pathNoSlashes][init:final]
                        plt.plot(ref['time'][init:final],
                                 y, ls=ls,
                                 color=self.colors[r],
                                 linewidth=2.5)
                        ymin = np.minimum(ymin, np.min(y))
                        ymax = np.maximum(ymax, np.max(y))

                # Plot the variable values from the MocoTrajectory.
                plt.plot(self.time, var, ls=ls, 
                         color=self.colors[len(self.refs)],
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
            from matplotlib.ticker import FormatStrFormatter
            ax.xaxis.set_major_formatter(
                FormatStrFormatter('%.1f'))

            # If we filled up the current figure or ran out of keys, add this
            # figure as a new page to the PDF. Otherwise, increment the plot
            # counter and keep going.
            if (p % self.plots_per_page == 0) or (i == len(var_dict.keys())-1):
                legfontsize = 64 / len(self.legend_handles)
                if legfontsize > 10: legfontsize = 10
                fig.tight_layout()
                plt.figlegend(self.legend_handles, self.legend_labels,
                              loc='lower center',
                              bbox_to_anchor=(0.5, 0.85),
                              fancybox=True, shadow=True,
                              prop={'size': legfontsize})
                self.pdf.savefig(fig)
                plt.close()
                p = 1
            else:
                p += 1

    def generate(self):

        ## Generate report.
        # =================
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_pdf import PdfPages

        with PdfPages(self.output) as self.pdf:

            # States & Accelerations
            # ----------------------
            state_names = self.trajectory.getStateNames()
            derivative_names = self.trajectory.getDerivativeNames()
            derivs = True if (len(derivative_names) > 0) else False
            accels = False
            auxiliary_derivative_names = list()
            for derivative_name in derivative_names:
                leaf = os.path.basename(os.path.normpath(derivative_name))
                if leaf == 'accel':
                    accels = True
                else:
                    auxiliary_derivative_names.append(derivative_name)

            if len(state_names) > 0:
                # Loop through the model's joints and cooresponding coordinates to
                # store plotting information.
                state_dict = OrderedDict()
                state_ls_dict = defaultdict(list)
                state_label_dict = dict()
                acceleration_dict = OrderedDict()
                acceleration_ls_dict = defaultdict(list)
                acceleration_label_dict = dict()
                coordSet = self.model.getCoordinateSet()
                for c in range(coordSet.getSize()):
                    coord = coordSet.get(c)
                    coordName = coord.getName()
                    coordPath = coord.getAbsolutePathString()
                    coordMotType = coord.getMotionType()
                    # Append suffixes to create names for position and speed state
                    # variables.
                    valueName = coordName + '/value'
                    speedName = coordName + '/speed'
                    if accels: accelName = coordName + '/accel'
                    if self.bilateral:
                        # If the --bilateral flag was set by the user, remove
                        # substrings that indicate the body side and update the
                        # linestyle dict.
                        valueName, state_ls_dict = bilateralize(valueName,
                                state_ls_dict)
                        speedName, state_ls_dict = bilateralize(speedName,
                                state_ls_dict)
                        if accels:
                            accelName, acceleration_ls_dict = bilateralize(
                                accelName, acceleration_ls_dict)

                    else:
                        state_ls_dict[valueName].append('-')
                        state_ls_dict[speedName].append('-')
                        if accels: acceleration_ls_dict[accelName].append('-')


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

                    if accels:
                        if not accelName in acceleration_dict:
                            acceleration_dict[accelName] = list()
                        acceleration_dict[accelName].append(coordPath + '/accel')
                        acceleration_label_dict[accelName] = \
                            getLabelFromMotionType(coordMotType, 'accel')

                self.plotVariables('state', state_dict, state_ls_dict, 
                        state_label_dict)
                if accels:
                    self.plotVariables('derivative', acceleration_dict,
                            acceleration_ls_dict, acceleration_label_dict)

                # Activations
                activ_dict = OrderedDict()
                activ_ls_dict = defaultdict(list)
                activ_label_dict = dict()
                for state_name in state_names:
                    if state_name.endswith('/activation'):
                        title = state_name
                        if self.bilateral:
                            title, activ_ls_dict = bilateralize(title,
                                                                activ_ls_dict)
                        else:
                            activ_ls_dict[title].append('-')
                        if not title in activ_dict:
                            activ_dict[title] = list()
                        # If --bilateral was set, the 'title' key will
                        # correspond to a list containing paths for both sides
                        # of the model.
                        activ_dict[title].append(state_name)
                        activ_label_dict[title] = ''
                self.plotVariables('state', activ_dict, activ_ls_dict,
                                   activ_label_dict)

                # Normalized tendon forces
                norm_tendon_force_dict = OrderedDict()
                norm_tendon_force_ls_dict = defaultdict(list)
                norm_tendon_force_label_dict = dict()
                for state_name in state_names:
                    if state_name.endswith('/normalized_tendon_force'):
                        title = state_name
                        if self.bilateral:
                            title, norm_tendon_force_ls_dict = bilateralize(
                                title, norm_tendon_force_ls_dict)
                        else:
                            norm_tendon_force_ls_dict[title].append('-')
                        if not title in norm_tendon_force_dict:
                            norm_tendon_force_dict[title] = list()
                        # If --bilateral was set, the 'title' key will
                        # correspond to a list containing paths for both sides
                        # of the model.
                        norm_tendon_force_dict[title].append(state_name)
                        norm_tendon_force_label_dict[title] = ''
                self.plotVariables('state', norm_tendon_force_dict,
                        norm_tendon_force_ls_dict,
                        norm_tendon_force_label_dict)

                # Auxiliary derivative variables
                aux_dict = OrderedDict()
                aux_ls_dict = defaultdict(list)
                aux_label_dict = dict()
                for aux_name in auxiliary_derivative_names:
                    title = aux_name
                    if self.bilateral:
                        title, aux_ls_dict = bilateralize(title,
                                                            aux_ls_dict)
                    else:
                        aux_ls_dict[title].append('-')
                    if not title in aux_dict:
                        aux_dict[title] = list()
                    # If --bilateral was set, the 'title' key will
                    # correspond to a list containing paths for both sides
                    # of the model.
                    aux_dict[title].append(aux_name)
                    aux_label_dict[title] = ''
                self.plotVariables('derivative', aux_dict, aux_ls_dict,
                                   aux_label_dict)
                
            # Input controls
            # --------------
            input_control_names = self.trajectory.getInputControlNames()
            if len(input_control_names) > 0:
                input_control_dict = OrderedDict()
                ls_dict = defaultdict(list)
                label_dict = dict()
                for input_control_name in input_control_names:
                    title = input_control_name.replace('/', '')
                    if self.bilateral:
                        # If the --bilateral flag was set by the user, remove
                        # substrings that indicate the body side and update the
                        # linestyle dict.
                        title, ls_dict = bilateralize(title, ls_dict)
                    else:
                        ls_dict[title].append('-')

                    if not title in input_control_dict:
                        input_control_dict[title] = list()
                    # If --bilateral was set, the 'title' key will correspond
                    # to a list containing paths for both sides of the model.
                    input_control_dict[title].append(input_control_name)
                    label_dict[title] = ''
                self.plotVariables('input_control', input_control_dict, ls_dict, 
                        label_dict)

            # Controls
            # --------
            control_names = self.trajectory.getControlNames()
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
            multiplier_names = self.trajectory.getMultiplierNames()
            if len(multiplier_names) > 0:
                multiplier_dict = OrderedDict()
                ls_dict = defaultdict(list)
                label_dict = dict()
                for multiplier_name in multiplier_names:
                    title = multiplier_name.replace('/', '')
                    if self.bilateral:
                        # If the --bilateral flag was set by the user, remove
                        # substrings that indicate the body side and update the
                        # linestyle dict.
                        title, ls_dict = bilateralize(multiplier_name, ls_dict)
                    else:
                        ls_dict[title].append('-')

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
            parameter_names = self.trajectory.getParameterNames()
            if len(parameter_names) > 0:
                fig = plt.figure(figsize=(8.5, 11))
                fig.patch.set_visible(False)
                ax = plt.axes()

                parameters = convert(self.trajectory.getParameters())
                cell_text = [['%.3f' % p] for p in parameters]

                plt.table(cellText=cell_text, rowLabels=parameter_names,
                          colLabels=[self.trajectory_fname], colWidths=[0.75], 
                          loc='center', cellLoc='center', rowLoc='center', 
                          colLoc='center', fontsize=16)
                ax.axis('off')
                ax.axis('tight')

                plt.subplots_adjust(left=0.2, right=0.8)

                legfontsize = 64 / len(self.legend_handles)
                if legfontsize > 10: legfontsize = 10
                plt.figlegend(self.legend_handles, self.legend_labels,
                              loc='lower center',
                              bbox_to_anchor=(0.5, 0.85),
                              fancybox=True, shadow=True,
                              prop={'size': legfontsize})
                self.pdf.savefig(fig)
                plt.close()

            # Slacks
            # ------
            # TODO slacks not accessible through MocoTrajectory
            # TODO should we even plot these?

def main():
    import argparse

    ## Input parsing.
    ## =============
    parser = argparse.ArgumentParser(
        description="Generate a report given a MocoTrajectory and an associated "
                    "OpenSim Model. Optionally, additional reference data "
                    "compatible with the MocoTrajectory may be plotted "
                    "simultaneously.")
    # Required arguments.
    parser.add_argument('model_or_mocostudy',
                        metavar='model-or-mocostudy', type=str,
                        help="OpenSim Model or MocoStudy file name "
                             "(including path).")
    parser.add_argument('trajectory', type=str,
                        help="MocoTrajectory file name (including path).")
    # Optional arguments.
    parser.add_argument('--bilateral', action='store_true',
                        help="Plot left and right limb states and controls "
                             "together.")
    parser.add_argument('--ref_files', type=str, nargs='+',
                        help="Paths to reference data files.")
    parser.add_argument('--colormap', type=str,
                        help="Matplotlib colormap from which plot colors are "
                             "sampled from.")
    parser.add_argument('--output', type=str,
                        help="Write the report to this filepath. "
                             "Default: the report is named "
                             "<trajectory-without-.sto>_report.pdf")
    args = parser.parse_args()

    # Load the Model and MocoTrajectory from file.
    model_or_mocostudy = args.model_or_mocostudy
    if model_or_mocostudy.endswith('.osim'):
        model = osim.Model(model_or_mocostudy)
    elif model_or_mocostudy.endswith('.omoco'):
        study = osim.MocoStudy(model_or_mocostudy)
        model = study.getProblem().createRep().getModelBase()

    ref_files = args.ref_files

    # Load the colormap provided by the user. Use a default colormap ('jet') 
    # if not provided. Uniformly sample the colormap based on the number of 
    # reference data sets, plus one for the MocoTrajectory.
    colors = list()
    refs = list()
    if ref_files != None: refs = ref_files
    colormap = args.colormap
    if colormap is None: colormap = 'jet'
    import matplotlib
    cmap_samples = np.linspace(0.1, 0.9, len(refs)+1)
    cmap = matplotlib.colormaps['jet']
    for sample in cmap_samples:
        colors.append(cmap(sample))

    report = Report(model=model,
                    trajectory_filepath=args.trajectory,
                    bilateral=args.bilateral,
                    colors=colors,
                    ref_files=ref_files,
                    output=args.output,
                    )
    report.generate()

if __name__ == "__main__":
    main()

