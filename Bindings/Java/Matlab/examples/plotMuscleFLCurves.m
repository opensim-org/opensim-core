function plotMuscleFLCurves(modelpath)
%% Function for computing and ploting the active and passive force--length 
%   curves for a specified muscle over the range of possible fiber lengths. 
%   This range is only approximate for muscles that cross more than one 
%   degree of freedom.
%   'modelpath' input is a full path string to an OpenSim model.

% -------------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  %
% See http://opensim.stanford.edu and the NOTICE file for more information.  %
% OpenSim is developed at Stanford University and supported by the US        %
% National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    %
% through the Warrior Web program.                                           %
%                                                                            %
% Copyright (c) 2005-2019 Stanford University and the Authors                %
% Author(s): James Dunne                                                     %
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

% Author: James Dunne, Tom Uchida, Chris Dembia, Ajay Seth.

%% Import OpenSim libraries.
import org.opensim.modeling.*

%%% If no model is input, get a path to one.
display('Loading the model...');
if nargin < 1
    [pathname,filename] = uigetfile('*.osim', 'Select an OpenSim Model File');
    modelpath = fullfile(filename,pathname);
elseif nargin > 1
    error('Too many inputs to function. Input is Model path');
end

%% Instantiate the model
model = Model(modelpath);

%% Instantiate the underlying computational System and return a handle to the State
state = model.initSystem();

% Ensure the model contains at least one muscle.
if (model.getMuscles().getSize() < 1)
    display('No muscles found; exiting.');
    return;
end

% Display all muscle names.
musclelist = {};
fprintf('%d muscles found:\n', model.getMuscles().getSize());
for i = 0 : model.getMuscles().getSize() - 1
    thisName = char(model.getMuscles().get(i).getName());
    musclelist = [ musclelist; {thisName} ];
    display([ '  ', thisName ]);
end

% Prompt the user to input a muscle name.
stopLoop = false;

while (~stopLoop)
    validName = false;

    while (~validName)
        string = input('Type a muscle name to plot, or ''exit'' to close: ', 's');

        % Get a valid name or 'exit'.
        nameIndex = ...
            find(cellfun(@(s) ~isempty(strmatch(string, s)), musclelist) == 1);

        if strmatch(string, 'exit');
            stopLoop = true;
            break;
        elseif isempty(nameIndex);
            display('Muscle name not found.');
        else
            validName = true;
            musclename = char( musclelist(nameIndex) );
        end
    end

    if (stopLoop)
        display('Exiting.');
        close all;
        return;
    end

    % Find the coordinates that the muscle crosses.
    muscle = getMuscleCoordinates(model, state, musclename);

    % Get the force--length curves of the muscle.
    [fl_active, fl_passive] = getForceLength(model, state, muscle);

    % Plot the results.
    fig = figure(1);
    clf(fig);

    % Make scatter plots for the active and passive components.
    hold on;
    title(musclename);
    scatter(fl_active(:,1), fl_active(:,2), 'b', ...
            'DisplayName', 'Active');
    scatter(fl_passive(:,1), fl_passive(:,2), 'r', ...
            'DisplayName', 'Passive');
    xlim([0.4, 1.6]);
    xlabel('Fiber Length (Normalised)');
    ylabel('Fiber Force (N)');
    legend('show');
    hold off;

end %while (~stopLoop)
end %function plotMuscleFLCurves

%% -----------------------------------------------------------------------------
function muscle = getMuscleCoordinates(model, state, muscleName)
%% Muscle coordinate finder
%   Returns a structure containing the coordinates that a muscle crosses and the
%   range of values for which the muscle can generate a moment. This is done by
%   examining the moment arm of the muscle across all coordinates in the model
%   and recording where the moment arm is nonzero.

import org.opensim.modeling.*  % Import OpenSim libraries.

%% Get a reference to the concrete muscle class.
force = model.getMuscles().get(muscleName);
muscleClass = char(force.getConcreteClassName());
eval(['muscle = ' muscleClass '.safeDownCast(force);']);

%% Initialize.
nCoord = model.getCoordinateSet().getSize();
muscCoord = [];  % For storing coordinate values.

%% Iterate through coordinates, finding nonzero moment arms.
for k = 0 : nCoord - 1
    % Get a reference to a coordinate.
    aCoord = model.getCoordinateSet().get(k);
    % Get coordinate's max and min values.
    rMax = aCoord.getRangeMax();
    rMin = aCoord.getRangeMin();
    rDefault = aCoord.getDefaultValue();
    % Define three points in the range to test the moment arm.
    totalRange = rMax - rMin;
    p(1) = rMin + totalRange/2;
    p(2) = rMin + totalRange/3;
    p(3) = rMin + 2*(totalRange/3);

    for i = 1 : 3
        aCoord.setValue(state, p(i));

        % Compute the moment arm of the muscle for this coordinate.
        momentArm = muscle.computeMomentArm(state, aCoord);

        % Avoid false positives due to roundoff error.
        tol = 1e-6;
        if ( abs(momentArm) > tol )
            muscCoord = [muscCoord; k];
            break;
        end
    end

    % Set the coordinate back to its default value.
    aCoord.setValue(state, rDefault);
end

%% Initialize the structure that will be returned.
muscle = struct();
muscle.name = muscleName;

%% Cycle through each coordinate found above and save its range of values. These
% will get used later to calculate muscle forces.
for u = 1 : length(muscCoord)
    % Get a reference to the coordinate.
    aCoord = model.getCoordinateSet().get(muscCoord(u));
    % Create an array of radian values for the range.
    coordRange = (aCoord.getRangeMin() : 0.01 : aCoord.getRangeMax())';
    % Store the coordinate and its range of values in the structure.
    eval(['muscle.coordinates.', ...
          char(model.getCoordinateSet().get(muscCoord(u))), ' = [coordRange];']);
end

end


%% -----------------------------------------------------------------------------
function [fl_active, fl_passive] = getForceLength(model, s, muscle)
% This function gets the active and passive force--length values across the
% possible fiber lengths of the muscle. fl_active and fl_passive are matrices
% containing forces corresponding to each fiber length.

import org.opensim.modeling.*  % Import OpenSim libraries.

% Get the number of coordinates for the muscle.
coordNames = fieldnames(muscle.coordinates);
nCoords = length( coordNames );

% Get a reference to the concrete muscle class.
force = model.getMuscles().get(muscle.name);
muscleClass = char(force.getConcreteClassName());
eval(['myMuscle = ' muscleClass '.safeDownCast(force);']);

% Initilize a matrix for storing the complete force--length curve.
flMatrix = zeros(1,3);

for k = 1 : nCoords

   % Get the name and range of the coordinate.
   updCoord = model.updCoordinateSet().get( char(coordNames(k)) );
   coordRange = muscle.coordinates.(coordNames{k});
   storageData = zeros( length(coordRange), 5 );

   % Loop through each value of the coordinate and compute the fiber length and
   % force of the muscle.
   for j = 1 : length( coordRange )

        % Set the coordinate value.
        coordValue = coordRange(j);
        updCoord.setValue(s, coordValue);
        updCoord.setSpeedValue(s, 0);

        % Set the activation and fiber length
        myMuscle.setActivation( s, 1 );
        myMuscle.setDefaultFiberLength( 0.01 );
        myMuscle.setFiberLength( s, myMuscle.getOptimalFiberLength() );

        % Equilibrate the muscle and tendon forces.
        model.equilibrateMuscles( s );

        % Store all the data in the result matrix. This is ineffecient, but
        % demonstrates what can be stored.
        storageData(j,:) = [...
            rad2deg(coordValue) ...                    % Coordinate value
            myMuscle.getFiberLength(s) ...             % Fiber length
            myMuscle.getNormalizedFiberLength(s) ...   % Normalized fiber length
            myMuscle.getActiveFiberForce(s) ...        % Active fiber force
            myMuscle.getPassiveFiberForce(s) ];        % Passive fiber force

        % Check for redundancy in fiber length.
        if isempty( find( myMuscle.getNormalizedFiberLength(s) == flMatrix(:,1), 1 ) )
            flMatrix = [flMatrix ; storageData(j,3:5)];
        end
    end

    % Reset the coordinate back to its default value.
    updCoord.setValue(s, updCoord.getDefaultValue());
end

fl_active = flMatrix(:,[1 2]);
fl_passive = flMatrix(:,[1 3]);

end