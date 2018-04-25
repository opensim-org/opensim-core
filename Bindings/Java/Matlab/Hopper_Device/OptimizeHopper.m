function [x, f] = OptimizeHopper()
% OPTIMIZEHOPPER
%   This function uses fmincon to optimize the model for hop height.
%   Requires the Optimization Toolbox.

%-----------------------------------------------------------------------%
% The OpenSim API is a toolkit for musculoskeletal modeling and         %
% simulation. See http://opensim.stanford.edu and the NOTICE file       %
% for more information. OpenSim is developed at Stanford University     %
% and supported by the US National Institutes of Health (U54 GM072970,  %
% R24 HD065690) and by DARPA through the Warrior Web program.           %
%                                                                       %
% Copyright (c) 2017 Stanford University and the Authors                %
% Author(s): Carmichael Ong,  Nick Bianco                               %
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

% Three variables: 1) passive stiffness 
%                  2) active max torque 
%                  3) hop timing
% See buildHopperFromSolution() for more details.
numVariables = 3; 

% Variable bounds
lb = zeros(numVariables, 1);
ub = ones(numVariables, 1);

% Initial guess
x0 = 0.25*ub;

% Optimize!
opts.TolFun = 0.01;
[x, f] = fmincon(@objective,x0,[],[],[],[],lb,ub,[],opts);

% Evaluate optimized solution
hopper = buildHopperFromSolution(x);
EvaluateHopper(hopper, true, true);

end

function hopper = buildHopperFromSolution(x)
% BUILDHOPPERFROMSOLUTION
%   Build the hopper with the passive device wrapped and the active device
%   unwrapped (behind the knee). The optimization variables 'x' determine 
%   the device properties and the timing of the hop.

import org.opensim.modeling.*;

% These parameters determine the stiffness in the passive device and the
% maximum force in the active device, both on a 0-100 scale. The range of
% values for each device correspond to the slider values in the GUI which
% can be accessed by running InteractiveHopper.m.
passiveParameter = 100*x(1);
activeParameter = 100*x(2);

% Construct controls from the last optimization variable, x(3). This
% variable is the time when the active device is turned off (which 
% releases the loaded spring) and the vastus muscle is turned to 
% coordinate a hop.
% For deviceControl and excitation, the first row contains time nodes
% and the second row contains control values.
deviceControl = [0.0 0.01 x(3) x(3)+0.01 5.0;
                 0.0 1.0  1.0  0.0       0.0];
excitation =    [0.0      x(3) x(3)+0.01 5.0;
                 0.0      0.0  1.0       1.0];
          
% Build the hopper. This is the same function called after a solution is
% is created in the InteractiveHopper GUI. The default muscle is "The
% Average Joe".
hopper = BuildInteractiveHopperSolution(...
            'muscleExcitation', excitation, ...
            'addPassiveDevice', true, ...
            'passivePatellaWrap', true, ...
            'passiveParameter', passiveParameter, ...
            'addActiveDevice', true, ...
            'activePatellaWrap', false, ...
            'activeParameter', activeParameter, ...
            'deviceControl', deviceControl, ...
            'isActivePropMyo', false, ...
            'printModelInfo', false);
end

function f = objective(x)

hopper = buildHopperFromSolution(x);
[peakHeight, ~] = EvaluateHopper(hopper, false, true);

% Negate peak height for minimization
f = -peakHeight;

end
