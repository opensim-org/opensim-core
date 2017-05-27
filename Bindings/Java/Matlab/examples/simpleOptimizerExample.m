% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2017 Stanford University and the Authors             %
% Author(s): Ayman Habib, Lorenzo Flores                                  %
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

% simpleOptimizationExample.m                                                        
% Author: Ayman Habib, Lorenzo Flores

import org.opensim.modeling.*

% Read in osim model
modelFile = strcat('testData',filesep,'Arm26_Optimize.osim');
%Create the Original OpenSim model from a .osim file
osimModel = Model(modelFile);
state = osimModel.initSystem;

coords = osimModel.getCoordinateSet();
coords.get('r_shoulder_elev').setValue(state, 0.0);

% Get the set of muscles that are in the original model
muscles = osimModel.getMuscles(); 
nMuscles = muscles.getSize();

for i = 0:nMuscles-1

	muscles.get(i).setActivation(state, 1.0);
	afl = ActivationFiberLengthMuscle.safeDownCast(muscles.get(i));
    afl.setFiberLength(state, .1);

end

elbowFlexCoord = osimModel.updCoordinateSet().get('r_elbow_flex');
elbowFlexCoord.setValue(state, 1.0);

osimModel.equilibrateMuscles(state);

%Set up optimization
coordMin = elbowFlexCoord.getRangeMin();
coordMax = elbowFlexCoord.getRangeMax();

% Control bounds must be specified as a column vector, every row
% corresponds to a control
lowerBound = coordMin*ones(1,1);
upperBound = coordMax*ones(1,1);

% Parameters to be passed in to the optimizer
params.model = osimModel;
params.state = state;

% knobs or coefficients that need to be computed by the optimizer
initialCoefficients = 1.0*ones(1,1);

%options = optimset('MaxIter',5000,'TolFun',0.00001);

% Call the native optimizer fminsearch
[f,fval,exitflag,output] = fminsearch(@(coeffs0) simpleOptimizerObjectiveFunction(coeffs0,params),...
	initialCoefficients);

disp(strcat('Optimization #iterations=',num2str(output.iterations),' joint angle = ',num2str(f*180/pi),', moment arm = ',num2str(-fval)));

