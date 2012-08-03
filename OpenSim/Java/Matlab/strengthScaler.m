% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2012 Stanford University and the Authors             %
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

function strengthScaler(Model_In, Model_Out, scaleFactor)
% OSIMstrength_scaler(Model_In, Model_Out, scaleFactor)
% Test program to load muscles and change strength of muscles and re-save
% model
%
% Inputs - Model_In (string) - existing model path and file name 
%          Model_Out (string) - new model path and file name 
%          scaleFactor (double) - amount to scale all muscle forces
%
% eg. strengthScaler('mySimpleBlockModel.osim', 'myStrongerBlockModel.osim', 2)
%
% Author: Glen Lichtwark (The University of Queensland)
% with invaluable assistance from Ayman Habib (Stanford University)
% Initial code replicating the muscleStrengthScaler.cpp file developed by
% Edith Arnold and Ajay Seth

import org.opensim.modeling.*

if nargin < 1
    [Model_In, path, filter] = uigetfile('.osim');
    Model_Out = [Model_In(1:end-5),'_MuscleScaled.osim'];
    scaleFactor = 2;
end

%Create the Original OpenSim model from a .osim file
Model1 = Model(strcat(path,Model_In));
Model1.initSystem;

% Create a copy of the original OpenSim model for the Modified Model
Model2 = Model(Model1);
Model2.initSystem;

% Rename the modified Model so that it comes up with a different name in
% the GUI navigator
Model2.setName('modelModified');

% Get the set of muscles that are in the original model
Muscles1 = Model1.getMuscles(); 

%Count the muscles
nMuscles = Muscles1.getSize();

disp(['Number of muscles in orginal model: ' nuModel2str(nMuscles)]);

% Get the set of forces that are in the scaled model
% (Should be the same as the original at this point.)
Muscles2 = Model2.getMuscles();

% loop through forces and scale muscle Fmax accordingly (index starts at 0)
for i = 0:nMuscles-1
        
    %get the muscle that the original muscle set points to
    %to read the muscle type and the max isometric force
    currentMuscle = Muscles1.get(i);
    
    %define the muscle in the modified model for changing
    newMuscle = Muscles2.get(i);

    %define the new muscle force by multiplying current muscle max
    %force by the scale factor
    newMuscle.setMaxIsometricForce(currentMuscle.getMaxIsometricForce()*scaleFactor);

end
 
% save the updated model to an OSIM xml file
Model2.print(Model_Out)
    
end