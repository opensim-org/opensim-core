% -------------------------------------------------------------------------- %
%                 OpenSim:  examplePolynomialPathFitter.m                    %
% -------------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  %
% See http://opensim.stanford.edu and the NOTICE file for more information.  %
% OpenSim is developed at Stanford University and supported by the US        %
% National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    %
% through the Warrior Web program.                                           %
%                                                                            %
% Copyright (c) 2005-2023 Stanford University and the Authors                %
% Author(s): Nicholas Bianco                                                 %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the "License"); you may    %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an "AS IS" BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %

% This example demonstrates how to use the PolynomialPathFitter class to create
% function-based representations of muscle-tendon lengths and moment arms using
% multivariate polynomial functions. Depending on your machine, this example
% may take several minutes or more to complete. You can view the progress of
% the fitting process in the 'opensim.log' file.
function examplePolynomialPathFitter

import org.opensim.modeling.*;

% Create the PolynomialPathFitter
% -------------------------------
fitter = PolynomialPathFitter();

% Set the model.
%
% The model should contain path-based force objects (e.g., Muscles) that use
% geometry-based paths (e.g., GeometryPath) to model path lengths and moment
% arms. The fitter will create a set of FunctionBasedPaths that use
% MultivariatePolynomialFunctions to model the path lengths and moment arms
% of the original model.
model = Model('subject_walk_scaled.osim');
model.initSystem();
modelName = char(model.getName());
fitter.setModel(ModelProcessor(model));

% Set the coordinate values table.
%
% The fitter will randomly sample around the coordinate values provided in the
% table to generate model configurations for which to compute path lengths and
% moment arms. This table has many more rows than are needed for the fitter to
% generate a good fit, so we will remove some of the rows to speed up the
% fitting process.
values = TimeSeriesTable('coordinates.sto');
times = values.getIndependentColumn();
for i = times.size():-1:1
    if mod(i, 5) ~= 0
        values.removeRowAtIndex(i);
    end
end
fitter.setCoordinateValues(TableProcessor(values));

% Configure optional settings
% ---------------------------
% Use these settings to modify the default settings to tailor the fitting
% process to your model and motion. See the documentation for
% PolynomialPathFitter for more options.

% Set a (relative) directory to where the fitting results will be saved.
% Files printed to this directory include the set of FunctionBasedPaths
% created by the fitter, the path lengths and moment arms computed for each
% model configuration, and the path lengths and moment arms fitted to the
% polynomial functions. File names will be prepended with the name of the
% model.
resultsDir = 'results';
fitter.setOutputDirectory(resultsDir);

% Set the maximum order of the polynomials used to fit the path lengths
% and moment arms. Higher order polynomials might lead to a better fit,
% but could increase the computational time required to evaluate the
% path length functions.
fitter.setMaximumPolynomialOrder(5);

% By default, coordinate values are sample around the nominal coordinate
% values using bounds of [-10, 10] degrees. You can set custom bounds for
% individual coordinates using the appendCoordinateSamplingBounds() method.
fitter.appendCoordinateSamplingBounds(...
    '/jointset/hip_r/hip_flexion_r', Vec2(-15, 15));
fitter.appendCoordinateSamplingBounds(...
    '/jointset/hip_l/hip_flexion_l', Vec2(-15, 15));

% Run the fitter
% --------------
% Information about each step fitting process will be printed to the
% console including the path length and moment arm RMS error for
% each force object and averaged across all force objects.
fitter.run();

% Plot the results
% ----------------
% Use the plotting functions below to visualize the results of the
% fitting process and determine if the fits are good enough for your needs,
% or if the model or fitting settings need to be modified.

% Plot the sampled coordinate values used to generate the path lengths
% and moment arms.
plotCoordinateSamples(resultsDir, modelName);

% Plot the path lengths and moment arms computed from the original model
% paths (blue) and the fitted polynomial paths (orange).
%
% For most muscles the fit is very good, but there are noticeable fitting
% errors in a few muscles (e.g., /forceset/gaslat_r and /forceset/glmax1_r).
% Errors like these usually arise from the fitting process struggling with
% discontinuities due from wrapping geometry issues in the original model.
% Depending on size of the errors, you may want to adjust the wrapping
% geometry in the original model and re-run the fitter.
plotPathLengths(resultsDir, modelName);
plotMomentArms(resultsDir, modelName);

% Evaluate the fitted functions on a 'new' trajectory
% ---------------------------------------------------
% You can use PolynomialPathFitter to evaluate a set of previously fitted
% FunctionBasedPaths on a new trajectory. This can be useful if you want to
% evaluate the path lengths and moment arms of a model on a trajectory that
% was not used to fit the functions. This example uses the same trajectory
% used to fit the functions, but you can replace it with any trajectory
% that has a set of coordinate values that are consistent with the model.
% Note that we do not need to use the 'fitter' object to call this function
% (i.e., it is a static function).
functionBasedPathsFile = ...
    fullfile(resultsDir, [modelName, '_FunctionBasedPathSet.xml']);
PolynomialPathFitter.evaluateFunctionBasedPaths(...
    model, TableProcessor(values), functionBasedPathsFile);

% Replacing the original paths with the fitted paths
% --------------------------------------------------
% You can use a ModelProcessor to replace the original paths in the model
% with the fitted paths. This can be useful if you want to use the fitted
% paths in a simulation or analysis tool but keep the original paths in the
% model file.
modelProcessor = ModelProcessor('subject_walk_scaled.osim');
modelProcessor.append(...
    ModOpReplacePathsWithFunctionBasedPaths(functionBasedPathsFile));
model = modelProcessor.process();
model.initSystem();
model.print('subject_walk_scaled_fitted_paths.osim');

end

% Plotting helper functions.
% --------------------------

function plotResults(original, fitted, sampled, sampledFitted, ...
    nRow, nCol, yLabel, scale, figName)

import org.opensim.modeling.*;

labels = original.getColumnLabels();
originalColor = 'blue';
fittedColor = [0.8500 0.3250 0.0980]; % orange

nPlots = nRow * nCol;
nFig = ceil(labels.size() / nPlots);
for iFig = 1:nFig
    figure;
    for iRow = 1:nRow
        for iCol = 1:nCol
            iPlot = (iRow - 1) * nCol + iCol;
            iLabel = iPlot + (iFig - 1) * nPlots;
            if iLabel < labels.size()
                label = char(labels.get(iLabel));
                subplot(nRow, nCol, iPlot);

                if ~isempty(sampled)
                    scatter(convertStdVectorDoubleToMat(sampled.getIndependentColumn()), ...
                        scale * sampled.getDependentColumn(label).getAsMat(), ...
                            'Marker', 'o', ...
                            'MarkerEdgeColor', originalColor, ...
                            'MarkerEdgeAlpha', 0.01, ...
                            'MarkerFaceColor', originalColor, ...
                            'MarkerFaceAlpha', 0.01, ...
                            'LineWidth', 0.05);
                    hold on;
                end

                if ~isempty(sampledFitted)
                    scatter(convertStdVectorDoubleToMat(sampledFitted.getIndependentColumn()), ...
                        scale * sampledFitted.getDependentColumn(label).getAsMat(), ...
                            'Marker', 'o', ...
                            'MarkerEdgeColor', fittedColor, ...
                            'MarkerEdgeAlpha', 0.01, ...
                            'MarkerFaceColor', fittedColor, ...
                            'MarkerFaceAlpha', 0.01, ...
                            'LineWidth', 0.05);
                    hold on;
                end

                times = convertStdVectorDoubleToMat(original.getIndependentColumn());
                plot(times, scale * original.getDependentColumn(label).getAsMat(), ...
                    'LineWidth', 3, 'Color', originalColor);
                xlim([times(1), times(end)]);
                title(label, 'FontSize', 10, 'Interpreter', 'none');

                if ~isempty(fitted)
                    plot(convertStdVectorDoubleToMat(fitted.getIndependentColumn()), ...
                        scale * fitted.getDependentColumn(label).getAsMat(), ...
                        'LineWidth', 3, 'Color', fittedColor);
                end

                if iRow == nRow
                    xlabel('time (s)');
                else
                    set(gca, 'XTickLabel', []);
                end

                if iCol == 1                  
                    ylabel(yLabel);
                else
                    set(gca, 'YTickLabel', []);
                end

                hold off;
            end
        end
    end

    sgtitle(figName, 'Interpreter', 'none');
end
end

function plotCoordinateSamples(resultsDir, modelName)
import org.opensim.modeling.*;

coordinates = TimeSeriesTable(...
    fullfile(resultsDir, [modelName, '_coordinate_values.sto']));
coordinatesSampled = TimeSeriesTable(...
    fullfile(resultsDir, [modelName, '_coordinate_values_sampled.sto']));
plotResults(coordinates, [], coordinatesSampled, [], ...
    3, 3, 'value (deg)', 180.0 / pi, 'Coordinate Values');
end

function plotPathLengths(resultsDir, modelName)
import org.opensim.modeling.*;

pathLengths = TimeSeriesTable(...
    fullfile(resultsDir, [modelName, '_path_lengths.sto']));
pathLengthsFitted = TimeSeriesTable(...
    fullfile(resultsDir, [modelName, '_path_lengths_fitted.sto']));
pathLengthsSampled = TimeSeriesTable(...
    fullfile(resultsDir, [modelName, '_path_lengths_sampled.sto']));
pathLengthsSampledFitted = TimeSeriesTable(...
    fullfile(resultsDir, [modelName, '_path_lengths_sampled_fitted.sto']));
plotResults(pathLengths, pathLengthsFitted, ...
    pathLengthsSampled, pathLengthsSampledFitted, 4, 4, ...
    'length (cm)', 100.0, 'Path Lengths');
end

function plotMomentArms(resultsDir, modelName)
import org.opensim.modeling.*;

momentArms = TimeSeriesTable(...
    fullfile(resultsDir, [modelName, '_moment_arms.sto']));
momentArmsFitted = TimeSeriesTable(...
    fullfile(resultsDir, [modelName, '_moment_arms_fitted.sto']));
momentArmsSampled = TimeSeriesTable(...
    fullfile(resultsDir, [modelName, '_moment_arms_sampled.sto']));
momentArmsSampledFitted = TimeSeriesTable(...
    fullfile(resultsDir, [modelName, '_moment_arms_sampled_fitted.sto']));
plotResults(momentArms, momentArmsFitted, ...
    momentArmsSampled, momentArmsSampledFitted, 4, 4, ...
    'moment arm (cm)', 100.0, 'Moment Arms');
end

function [mat] = convertStdVectorDoubleToMat(vec)

mat = zeros(vec.size(), 1);
for i = 1:vec.size()
    mat(i) = vec.get(i-1);
end

end
