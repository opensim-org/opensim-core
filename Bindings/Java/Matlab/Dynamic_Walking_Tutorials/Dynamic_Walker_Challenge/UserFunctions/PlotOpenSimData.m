%% PlotOpenSimData
%   Plots the results of a Passive Dynamic Walker Simulation. In
%   particular, the Pelvis Tx, Right Hip, and Right Knee Coordinates.

% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
% Author(s): James Dunne
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% http://www.apache.org/licenses/LICENSE-2.0.
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
% implied. See the License for the specific language governing
% permissions and limitations under the License.
% -----------------------------------------------------------------------

%% Import OpenSim Libraries
import org.opensim.modeling.*

%% Find states from Walker Simulation
if ~exist('ResultsFWD/simulation_states.sto','file')
    warning('Cannot find ResultsFWD/simulation_states.sto, please select file')
    [filename,pathname] = uigetfile('*.sto', 'Select simulation_states.sto file');
    filepath = fullfile(pathname, filename);
else
    filepath = fullfile(cd, 'ResultsFWD/simulation_states.sto');
end

%% Use the OpenSim TimeSeriesTable to load the data into an OpenSim Table
opensimTable = TimeSeriesTable(filepath);

%% Get the relevent data from the file
% Define the coordinates of interest by name
coordinatesOfInterest = [{'Pelvis_tx/value'} {'RHip_rz/value'} {'RKnee_rz/value'}];
% Pre-allocate some arrays
plotData = zeros( opensimTable.getNumRows(), 3);
timeArray = zeros(opensimTable.getNumRows(),1);

% Get the column labels from the TimeSeriesTable
labels = opensimTable.getColumnLabels();

% Get the coordinate data
for i = 0 : labels.size() - 1
    for u = 1 : 3
        if contains(char(labels.get(i)),coordinatesOfInterest{u})
            for k = 0 : opensimTable.getNumRows() - 1
                plotData(k+1,u) = opensimTable.getDependentColumnAtIndex(i).get(k);
            end
        end
    end
end

% Get the time array
for k = 0 : opensimTable.getNumRows() - 1
    timeArray(k+1,1) = opensimTable.getIndependentColumn().get(k);
end

%% Generate plots for the data
% Set up plot parameters
colorOpts = {'b', 'g', 'r'};
lineWidth = 2.5;

% Create figure
figHandle = figure;
hold on
for i = 1 : 3
    plothandle = plot(timeArray,plotData(:,i),...
                    'Color',colorOpts{i},...
                    'linewidth', lineWidth);
end
% Add a legend to the plot
legend(coordinatesOfInterest')
% Add a title
title('Dynamic Walker Simulation Results')
% Add Axis Labels
xlabel('Time (Seconds)')
ylabel('Coordinate Value (Rotations in Radians)')
hold off
