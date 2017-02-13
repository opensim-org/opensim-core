% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2017 Stanford University and the Authors             %
% Author(s): James Dunne                                                  %
%                                                                         %
% Licensed under the Apache License, Version 2.0 (the "License");         %
% you may not use this file except in compliance with the License.        %
% You may obtain a copy of the License at                                 %
% http://www.apache.org/licenses/LICENSE-2.0.                             %
%                                                                         %
% Unless required by applicable law or agreed to in writing, software     %
% distributed under the License is distributed on an "AS IS" BASIS,       %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         %
% implied. See the License for the specific language governing            %
% permissions and limitations under the License.                          %
% ----------------------------------------------------------------------- %

% Author: James Dunne, Chris Dembia, Tom Uchida, Ajay Seth.

function [fl_active,fl_passive] = plotMuscleFLCurves(modelpath)
% modelpath input is a full path string to an OpenSim model.

% Import OpenSim Libraries
import org.opensim.modeling.*      

if nargin < 1
    [filein, pathname] = uigetfile({'*.osim','osim'}, 'OSIM model file...');
    model = Model(fullfile(pathname,filein));
elseif nargin == 1
    model = Model(modelpath);
end

s =  model.initSystem();


%% select a muscle
musclelist = {};
name = [];
stoploop = [];

for i = 0 : model.getMuscles.getSize() - 1
    musclelist = [ musclelist ; {char(model.getMuscles.get(i).getName)} ];
    display( char(model.getMuscles.get(i).getName ) )
end
display('(type exit to stop).');


while isempty(stoploop)

    %%%
    while isempty(name)

        string = input('Type a muscle name to plot, or exit to close; ', 's');

        % get a valid name or 'exit'
        nameIndex = find( cellfun(@(s) ~isempty(strmatch(string, s)), musclelist) == 1);

        if strmatch(string, 'exit');
            stoploop = 1;
            break
        elseif isempty(nameIndex);
            display('Muscle name not found.');
        else
            name = 1;
            musclename = char( musclelist(nameIndex) );
        end
    end
    %%%

    if ~isempty(stoploop)
        display('thanks for ending this program. See you Laterz!')
        close all
        clc
        return
    end

    %% Muscle Coordinate finder
    %   Find the coordinates that each muscle crosses. This is done by
    %   examining the moment arm contribution of the muscle across all
    %   coordinates. A muscle will contribute to any coodinate when the moment
    %   arm is non-zero.
    muscle = getMusclecoordinates( model , s, musclename);

    % Get the force length curves of a muscle
    [fl_active,fl_passive] = getForceLength(model, s, muscle);

    %% plot the results
    fig = figure(1);
    clf(fig)
    hold

    % Make scatter plots foe the active and passive components
    scatter(fl_active(:,1),fl_active(:,2),'b','DisplayName','Active Fiber Force')
    scatter(fl_passive(:,1),fl_passive(:,2),'r','DisplayName','Active Passive Force')
    % Limit the X axis from 0.4 to 1.6
    xlim([0.4 1.6]);
    
    xlabel('Force (N)')
    ylabel('Fiber Length (Normalised')
    
    
    hold off
    legend('show')

    % clear the muscle name.
    name =[];
end




end



function muscle = getMusclecoordinates(model,state,muscleName)
% Muscle Coordinate finder
%   Returns a structure with the coordinates, and coordinate ranges, that a 
%   a muscle effects. This is done by
%   examining the moment arm contribution of the muscle across all
%   coordinates. A muscle will contribute to any coodinate when the moment
%   arm is non-zero.

import org.opensim.modeling.*      % Import OpenSim Libraries

% get the muscleCoordinates type by getting the concrete Class Name
force = model.getMuscles().get(muscleName);

% get a reference to the concrete muscle class in the model
muscleType = char(force.getConcreteClassName() );
eval(['muscle =' muscleType '.safeDownCast(force);'])

% get a fresh matrix to dump coordinate values into
nCoord = model.getCoordinateSet.getSize();
muscCoord =[];

% iterate through coordinates, finding non-zero moment arms
for k = 0 : nCoord -1
    % get a reference to a coordinate
    aCoord = model.getCoordinateSet.get(k);
    % get coordinate Max and Min
    rMax = aCoord.getRangeMax();
    rMin = aCoord.getRangeMin();
    rDefaut = aCoord.getDefaultValue();
    % define three points in the range to test that the moment arm is
    % non-zero
    totalRange = rMax - rMin;
    p(1) = rMin + totalRange/2;
    p(2) = rMin + totalRange/3;
    p(3) = rMin + 2*(totalRange/3);

    for i = 1 : 3

        aCoord.setValue(state, p(i) );

        % compute the moment arm of the muscle for that coordinate given
        % the state.
        momentArm = muscle.computeMomentArm(state,aCoord);

        % round the numbers. This is needed because at some coordinates there
        % are moments generated at the e-18 level. This is most likely
        % numerical error. So to deal with this I round to 4 decimal points.
        tol = 1e-6;
        if ( abs(momentArm) > tol )
            muscCoord = [muscCoord; k];
            break
        end
    end
    % Set the coordinate value back to the default. 
    aCoord.setValue(state, rDefaut );
end

muscle = struct();
% Cycle through each available coordinate and save its range values.
% These will get used later to run calculate muscle force on each
% coordinate value.
for u = 1 : length(muscCoord)
    % Get a reference to the coordinate
    aCoord = model.getCoordinateSet.get(muscCoord(u));
    % Create an arrary of radian value's for the range
    coordRange = (aCoord.getRangeMin:0.01:aCoord.getRangeMax())';
    % add the coordinates and their range values to the structure
    eval(['muscle.coordinates.' char(model.getCoordinateSet.get(muscCoord(u))) ' = [coordRange];' ])

end
muscle.name = muscleName;
end


function [fl_active,fl_passive] = getForceLength(model, s, muscle)
%  Function gets the active and passive force length values across the
%  possible fiber lengths of the muscle in the model. fl_active and
%  fl_passive are a matrices of fiberlength-force pairs. 

import org.opensim.modeling.*      % Import OpenSim Libraries

coordNames = fieldnames(muscle.coordinates);
% get the number of coordinates for the muscle
nCoords = length( coordNames );

% Get the muscle that is needed
force = model.getMuscles().get(muscle.name);
% Get the muscleType of that force
muscleType = char(force.getConcreteClassName());
% Get a reference to the concrete muscle class in the model
eval(['myMuscle =' muscleType '.safeDownCast(force);'])

% Initilize a matrix for storing the complete fl curve;
flMatrix = zeros(1,3);

for k = 1 : nCoords

   % Get the name of the coordinate
   aCoord = model.getCoordinateSet().get( char(coordNames(k)) );
   updCoord = model.updCoordinateSet().get( char(coordNames(k)) );
   coordRange = muscle.coordinates.(coordNames{k});
   storageData = zeros( length(coordRange), 5 );


   % Loop through each coordinate value and get the fiber
   % length and force of the muscle.
   for j = 1 : length( coordRange )

        % Get a current coordinate value
        coordValue = coordRange(j);
        updCoord.setValue(s, coordValue);
        updCoord.setSpeedValue(s, 0 );

        % Set the activation and fiber length
        myMuscle.setActivation( s, 1 )
        myMuscle.setDefaultFiberLength( 0.01 )
        myMuscle.setFiberLength( s, myMuscle.getOptimalFiberLength() )
        % Equilibrate the forces from the activation
        model.equilibrateMuscles( s );

        % Store all the data in the result matrix. This is ineffecient, but
        % shows what can be stored. 
        storageData(j,:) = [...
            rad2deg(coordValue) ...                     \% Coordinate Value
            myMuscle.getFiberLength(s) ...             % Fiber length
            myMuscle.getNormalizedFiberLength(s) ...   % Normalized Fiber Length
            myMuscle.getActiveFiberForce(s) ...        % Active Force
            myMuscle.getPassiveFiberForce(s) ];        % passive fiber forces

        % Check for redundancy in fiber length
        if isempty( find( myMuscle.getNormalizedFiberLength(s) == flMatrix(:,1), 1 ) )
            flMatrix = [flMatrix ; storageData(j,3:5)];
        end
   end

   aCoord.getDefaultValue();

    % Reset the coordinate value back to zero
    updCoord.setValue(s, aCoord.getDefaultValue());
end

fl_active = flMatrix(:,1:2);
fl_passive = flMatrix(:,[1 3]);

% sometimes there is a zero value that is incorrectly stored. It is abug
% this is a slack-patch. 
fl_active(find(fl_active(:,1) == 0 ),: ) = [];
fl_passive(find(fl_passive(:,1) == 0 ),: ) = [];

end
