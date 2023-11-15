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

import os
import opensim as osim
from examplePolynomialPathFitter_plotting import (plot_coordinate_samples,
                                                  plot_path_lengths,
                                                  plot_moment_arms)

# This example demonstrates how to use the PolynomialPathFitter class to create
# function-based representations of muscle-tendon lengths and moment arms using
# multivariate polynomial functions.

# Create the PolynomialPathFitter
# -------------------------------
fitter = osim.PolynomialPathFitter()

# Set the model.
#
# The model should contain path-based force objects (e.g., Muscles) that use
# geometry-based paths (e.g., GeometryPath) to model path lengths and moment
# arms. The fitter will create a set of FunctionBasedPaths that use
# MultivariatePolynomialFunctions to model the path lengths and moment arms
# of the original model.
model = osim.Model('subject_walk_scaled.osim')
model.initSystem()
fitter.setModel(osim.ModelProcessor(model))

# Set the coordinate values table.
#
# The fitter will randomly sample around the coordinate values provided in the
# table to generate model configurations for which to compute path lengths and
# moment arms. This table has many more rows than are needed for the fitter to
# generate a good fit, so we will remove some of the rows to speed up the
# fitting process.
values = osim.TimeSeriesTable('coordinates.sto')
times = values.getIndependentColumn()
for i in range(len(times)):
    if i % 5 != 0:
        values.removeRow(times[i])

fitter.setCoordinateValues(osim.TableProcessor(values))

# Configure optional settings
# ---------------------------
# Use these settings to modify the default settings to tailor the fitting
# process to your model and motion. See the documentation for
# PolynomialPathFitter for more options.

# Set a (relative) directory to where the fitting results will be saved.
# Files printed to this directory include the set of FunctionBasedPaths
# created by the fitter, the path lengths and moment arms computed for each
# model configuration, and the path lengths and moment arms fitted to the
# polynomial functions. File names will be prepended with the name of the
# model.
results_dir = 'results'
fitter.setOutputDirectory(results_dir)

# Set the maximum order of the polynomials used to fit the path lengths
# and moment arms. Higher order polynomials might lead to a better fit,
# but could increase the computational time required to evaluate the
# path length functions.
fitter.setMaximumPolynomialOrder(5)

# By default, coordinate values are sample around the nominal coordinate
# values using bounds of [-10, 10] degrees. You can set custom bounds for
# individual coordinates using the appendCoordinateSamplingBounds() method.
fitter.appendCoordinateSamplingBounds(
    '/jointset/hip_r/hip_flexion_r', osim.Vec2(-15, 15))
fitter.appendCoordinateSamplingBounds(
    '/jointset/hip_l/hip_flexion_l', osim.Vec2(-15, 15))

# Run the fitter
# --------------
# Information about each step fitting process will be printed to the
# console including the path length and moment arm RMS error for
# each force object and averaged across all force objects.
fitter.run()

# Plot the results
# ----------------
# Use the plotting functions in plotting.py to visualize the results of the
# fitting process and determine if the fits are good enough for your needs,
# or if the model or fitting settings need to be modified.

# Plot the sampled coordinate values used to generate the path lengths
# and moment arms.
plot_coordinate_samples(results_dir, model.getName())

# Plot the path lengths and moment arms computed from the original model
# paths (blue) and the fitted polynomial paths (orange).
#
# For most muscles the fit is very good, but there are noticeable fitting
# errors in a few muscles (e.g., /forceset/gaslat_r and /forceset/glmax1_r).
# Errors like these usually arise from the fitting process struggling with
# discontinuities due from wrapping geometry issues in the original model.
# Depending on size of the errors, you may want to adjust the wrapping
# geometry in the original model and re-run the fitter.
plot_path_lengths(results_dir, model.getName())
plot_moment_arms(results_dir, model.getName())

# Evaluate the fitted functions on a 'new' trajectory
# ---------------------------------------------------
# You can use PolynomialPathFitter to evaluate a set of previously fitted
# FunctionBasedPaths on a new trajectory. This can be useful if you want to
# evaluate the path lengths and moment arms of a model on a trajectory that
# was not used to fit the functions. This example uses the same trajectory
# used to fit the functions, but you can replace it with any trajectory
# that has a set of coordinate values that are consistent with the model.
# Note that we do not need to use the 'fitter' object to call this function
# (i.e., it is a static function).
functionBasedPathsFile = os.path.join(
    results_dir, f'{model.getName()}_FunctionBasedPathSet.xml')
osim.PolynomialPathFitter.evaluateFunctionBasedPaths(
    model, osim.TableProcessor(values), functionBasedPathsFile)

# Replacing the original paths with the fitted paths
# --------------------------------------------------
# You can use a ModelProcessor to replace the original paths in the model
# with the fitted paths. This can be useful if you want to use the fitted
# paths in a simulation or analysis tool but keep the original paths in the
# model file.
modelProcessor = osim.ModelProcessor('subject_walk_scaled.osim')
modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths(
    functionBasedPathsFile))
model = modelProcessor.process()
model.initSystem()
model.printToXML('subject_walk_scaled_fitted_paths.osim')
