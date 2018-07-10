function [func] = InteractiveHopperParameters(name)
% INTERACTIVEHOPPERPARAMETERS
%   Built-in parameters for the InteractiveHopper GUI example.

%-----------------------------------------------------------------------%
% The OpenSim API is a toolkit for musculoskeletal modeling and         %
% simulation. See http://opensim.stanford.edu and the NOTICE file       %
% for more information. OpenSim is developed at Stanford University     %
% and supported by the US National Institutes of Health (U54 GM072970,  %
% R24 HD065690) and by DARPA through the Warrior Web program.           %
%                                                                       %
% Copyright (c) 2017 Stanford University and the Authors                %
% Author(s): Nick Bianco, Carmichael Ong                                %
%                                                                       %
% Licensed under the Apache License, Version 2.0 (the "License");       %
% you may not use this file except in compliance with the License.      %
% You may obtain a copy of the License at                               %
% http://www.apache.org/licenses/LICENSE-2.0.                           %
%                                                                       %
% Unless required by applicable law or agreed to in writing, software   %
% distributed under the License is distributed on an "AS IS" BASIS,     %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or       %
% implied. See the License for the specific language governing          %
% permissions and limitations under the License.                        %
%-----------------------------------------------------------------------%

% Throw error if invalid parameter name is passed
try 
   func = getParameterFunction(name);
catch
   ME = MException('InteractiveHopperParameters:invalidParameterName', ...
                   'Input ''%s'' is not a valid InteractiveHopper parameter.', name);
   throw(ME);
end

% GETPARAMFUNCTION
%   Return parameter function handle for specified name.
function [func] = getParameterFunction(name)

switch name
    case 'controls'
        func = @controls;
    case 'passiveSlider'
        func = @passiveSlider;
    case 'activeSlider'
        func = @activeSlider;
    case 'defaultMuscle'
        func = 'averageJoe';
    case 'averageJoe'
        func = @averageJoe;
    case 'arnold'
        func = @arnold;
    case 'katieLedecky'
        func = @katieLedecky;
    case 'passive'
        func = @passive;
    case 'activeControl'
        func = @activeControl;
    case 'activePropMyo'
        func = @activePropMyo;
end

% CONTROLS
%   Function to return parameters for default muscle and device controls
%   and plotting colors.
function [muscleExcitation,muscleExcitationColor,deviceControl,deviceControlColor] = controls()
    
muscleExcitation = [0.0 1.99 2.0 3.89 3.9 5.0;
                    0.3 0.3  1.0 1.0  0.1 0.1];
muscleExcitationColor = [0.64 0.08 0.18];
deviceControl = [0.0 2.5 5.0;
                 0.0 0.75 0.0];
deviceControlColor = [0 0.45 0.74];

% PASSIVESLIDER
%   Function to set default, minimum, and maximum values of the 
%   passive device slider.
function [value,min,max] = passiveSlider()

value = 50;
min = 1;
max = 100;

% ACTIVESLIDER
%   Function to set default, minimum, and maximum values of the 
%   active device sliders.
function [value,min,max] = activeSlider()

value = 50;
min = 1;
max = 100;

% PASSIVE
%   Function to determine mass and spring stiffness of the passive device. 
%   In the InteractiveHopper GUI example, 'param' reflects the prescribed 
%   slider value for the passive device.
function [mass,stiffness] = passive(param)

stiffness = param*370;
mass = param*0.025;

% ACTIVECONTROL
%   Function to determine mass and maximum tension of the active device 
%   (with a unique user specified control). In the InteractiveHopper GUI 
%   example, 'param' reflects the prescribed slider value for the passive
%   device.
function [mass,maxTension] = activeControl(param)

maxTension = param*20;
mass = param*0.025;

% ACTIVEPROPMYO
%   Function to determine mass and maximum tension of the active device 
%   (with a proportional myoelectric controller). In the InteractiveHopper GUI 
%   example, 'param' reflects the prescribed slider value for the passive
%   device.
function [mass,gain] = activePropMyo(param)

gain = param/100;
mass = param*0.025;

% AVERAGEJOE
%   Settings for "The Average Joe" muscle in the InteractiveHopper GUI
%   example.
function [maxIsometricForce,optimalFiberLength,tendonSlackLength,mass] = averageJoe()

maxIsometricForce = 4000.0;
optimalFiberLength = 0.50;
tendonSlackLength = 0.30;
mass = 4.0;

% ARNOLD
%   Settings for "The Arnold" muscle in the InteractiveHopper GUI
%   example.
function [maxIsometricForce,optimalFiberLength,tendonSlackLength,mass] = arnold()

maxIsometricForce = 5000.0;
optimalFiberLength = 0.40;
tendonSlackLength = 0.40;
mass = 5.0;

% KATIELEDECKY
%   Settings for "The Katie Ledecky" muscle in the InteractiveHopper GUI
%   example.
function [maxIsometricForce,optimalFiberLength,tendonSlackLength,mass] = katieLedecky()

maxIsometricForce = 3500.0;
optimalFiberLength = 0.60;
tendonSlackLength = 0.20;
mass = 2.0;
