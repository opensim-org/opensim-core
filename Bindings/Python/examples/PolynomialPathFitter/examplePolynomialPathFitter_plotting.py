# -------------------------------------------------------------------------- #
#                 OpenSim:  examplePolynomialPathFitter.py                   #
# -------------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  #
# See http://opensim.stanford.edu and the NOTICE file for more information.  #
# OpenSim is developed at Stanford University and supported by the US        #
# National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    #
# through the Warrior Web program.                                           #
#                                                                            #
# Copyright (c) 2005-2023 Stanford University and the Authors                #
# Author(s): Nicholas Bianco                                                 #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License"); you may    #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #

# Plotting helper functions for examplePolynomialPathFitter.py.

import os
import opensim as osim
import numpy as np
from matplotlib import pyplot as plt


def plot_results(original, fitted=None, sampled=None, sampled_fitted=None,
                 nrow=4, ncol=4, ylabel='value', scale: float = 1):
    labels = original.getColumnLabels()
    original_color = 'blue'
    fitted_color = 'orange'

    nplots = nrow * ncol
    nfig = int(np.ceil(len(labels) / nplots))
    for ifig in range(nfig):
        fig, axs = plt.subplots(nrow, ncol, figsize=(10, 8))
        for irow in range(nrow):
            for icol in range(ncol):
                iplot = irow * ncol + icol
                ilabel = iplot + ifig * nplots
                if ilabel < len(labels):

                    if sampled is not None:
                        axs[irow, icol].scatter(
                            sampled.getIndependentColumn(),
                            scale * sampled.getDependentColumn(labels[ilabel]).to_numpy(),
                            alpha=0.15, color=original_color, s=0.5)

                    if sampled_fitted is not None:
                        axs[irow, icol].scatter(
                            sampled_fitted.getIndependentColumn(),
                            scale * sampled_fitted.getDependentColumn(labels[ilabel]).to_numpy(),
                            alpha=0.15, color=fitted_color, s=0.5)

                    times = original.getIndependentColumn()
                    axs[irow, icol].plot(
                        times,
                        scale * original.getDependentColumn(labels[ilabel]).to_numpy(),
                        lw=3, color=original_color)
                    axs[irow, icol].set_xlim(times[0], times[-1])
                    axs[irow, icol].set_title(labels[ilabel], fontsize=6)

                    if fitted is not None:
                        axs[irow, icol].plot(
                            fitted.getIndependentColumn(),
                            scale * fitted.getDependentColumn(labels[ilabel]).to_numpy(),
                            lw=3, color=fitted_color)

                    if irow == nrow - 1:
                        axs[irow, icol].set_xlabel('time (s)')
                    else:
                        axs[irow, icol].set_xlabel('')
                        axs[irow, icol].set_xticklabels([])

                    if icol == 0:
                        axs[irow, icol].set_ylabel(ylabel)

    plt.tight_layout()
    plt.show()


def plot_coordinate_samples(results_dir, model_name):
    coordinates = osim.TimeSeriesTable(
        os.path.join(results_dir, f'{model_name}_coordinate_values.sto'))
    coordinates_sampled = osim.TimeSeriesTable(
        os.path.join(results_dir, f'{model_name}_coordinate_values_sampled.sto'))
    plot_results(coordinates,
                 sampled=coordinates_sampled,
                 nrow=3, ncol=3, ylabel='value (deg)', scale=(180.0 / np.pi))


def plot_path_lengths(results_dir, model_name):
    path_lengths = osim.TimeSeriesTable(
        os.path.join(results_dir, f'{model_name}_path_lengths.sto'))
    path_lengths_fitted = osim.TimeSeriesTable(
        os.path.join(results_dir, f'{model_name}_path_lengths_fitted.sto'))
    path_lengths_sampled = osim.TimeSeriesTable(
        os.path.join(results_dir, f'{model_name}_path_lengths_sampled.sto'))
    path_lengths_sampled_fitted = osim.TimeSeriesTable(
        os.path.join(results_dir, f'{model_name}_path_lengths_sampled_fitted.sto'))
    plot_results(path_lengths,
                 fitted=path_lengths_fitted,
                 sampled=path_lengths_sampled,
                 sampled_fitted=path_lengths_sampled_fitted,
                 nrow=4, ncol=4, ylabel='length (cm)', scale=100.0)


def plot_moment_arms(results_dir, model_name):
    moment_arms = osim.TimeSeriesTable(
        os.path.join(results_dir, f'{model_name}_moment_arms.sto'))
    moment_arms_fitted = osim.TimeSeriesTable(
        os.path.join(results_dir, f'{model_name}_moment_arms_fitted.sto'))
    moment_arms_sampled = osim.TimeSeriesTable(
        os.path.join(results_dir, f'{model_name}_moment_arms_sampled.sto'))
    moment_arms_sampled_fitted = osim.TimeSeriesTable(
        os.path.join(results_dir, f'{model_name}_moment_arms_sampled_fitted.sto'))
    plot_results(moment_arms,
                 fitted=moment_arms_fitted,
                 sampled=moment_arms_sampled,
                 sampled_fitted=moment_arms_sampled_fitted,
                 nrow=4, ncol=4, ylabel='moment arm (cm)', scale=100.0)

