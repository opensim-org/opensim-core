function strengthScalar(Model_In, Model_Out, scaleFactor)
% OSIMstrength_scalar(Model_In, Model_Out, scaleFactor)
% Test program to load muscles and change strength of muscles and re-save
% model
%
% Inputs - Model_In (string) - existing model path and file name 
%          Model_Out (string) - new model path and file name 
%          scaleFactor (double) - amount to scale all muscle forces
%
% eg. strengthScalar('mySimpleBlockModel.osim', 'myStrongerBlockModel.osim', 2)
%
% Author: Glen Lichtwark (The University of Queensland)
% with invaluable assistance from Ayman Habib (Stanford University)
% Initial code replicating the muscleStrengthScalar.cpp file developed by
% Edith Arnold and Ajay Seth

import org.opensim.modeling.*

if nargin < 1
    Model_In = uigetfile('.osim');
    Model_Out = [Model_In(1:end-5),'_MuscleScaled.osim'];
    scaleFactor = 2;
end

%Create the Original OpenSim model from a .osim file
M1 = Model(Model_In);
M1.initSystem;

% Create a copy of the original OpenSim model for the Modified Model
M2 = Model(M1);
M2.initSystem;

% Rename the modified Model so that it comes up with a different name in
% the GUI navigator
M2.setName('modelModified');

% Get the set of forces that are in the original model
M1 = M1.getMuscles(); 
%Count the muscles
nMuscles = M1.getSize();

disp(['Number of muscles in orginal model: ' num2str(nMuscles)]);
% Get the set of forces that are in the scaled model
% (Should be the same as the original at this point.)
M2 = M2.getMuscles();

% loop through forces and scale muscle Fmax accordingly
for i = 1:nMuscles
        
    %get the muscle that the original muscle set points to
    %to read the muscle type and the max isometric force
    currentMuscle = M1.get(i-1);
    muscleType = currentMuscle.getType;
    disp([char(currentMuscle.getName()) '--> muscle type: ' char(muscleType)]);

    %define the muscle in the modified model for changing
    newMuscle = M2.get(i-1);

    %define the new muscle force by multiplying current muscle max
    %force by the scale factor
    newMuscle.setMaxIsometricForce(currentMuscle.getMaxIsometricForce()*scaleFactor);

end
 
% save the updated model to an OSIM xml file
M2.print(Model_Out)
    
end