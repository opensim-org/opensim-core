function [x, f] = OptimizeHopper()
% OPTIMIZEHOPPER
%   This function implements two different optimization approaches to 
%   maximize the jump height of the hopper.
%
%-----------------------------------------------------------------------%
% The OpenSim API is a toolkit for musculoskeletal modeling and         %
% simulation. See http://opensim.stanford.edu and the NOTICE file       %
% for more information. OpenSim is developed at Stanford University     %
% and supported by the US National Institutes of Health (U54 GM072970,  %
% R24 HD065690) and by DARPA through the Warrior Web program.           %
%                                                                       %
% Copyright (c) 2017 Stanford University and the Authors                %
% Author(s): Nicholas Bianco, Carmichael Ong                            %
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

% CHOOSE PROBLEM: 'one_hop' or 'three_hops'
problem = 'one_hop';
switch problem
    
    % Optimize the height of one hop. The single optimization variable 
    % determines when to simultaneously release a loaded passive device and 
    % activate the hopper vastus muscle (see CONSTRUCTCONTROLS below). This
    % simple univariate problem uses FMINBND from the MATLAB Optimization 
    % Toolbox. 
    case 'one_hop'
        lb = 0;
        ub = 1;
        options = optimset('TolFun', 1e-2, ...
                           'MaxIter', 10000, ...
                           'Display', 'iter'); 
        [x, f] = fminbnd(@(x) objective(x,problem),lb,ub,options);
                     
    % Optimize the height of a sequence of three hops. The optimization
    % variables decide to switch the actuators on and off to load and 
    % release the passive device with the activated muscle (see 
    % CONSTRUCTCONTROLS below). This problem is subject to falling into
    % undesired local minima (e.g. solutions with less than three hops) and
    % requires an optimizer from the MATLAB Global Optimization Toolbox.
    % Here we use the genetic algorithm optimizer, GA.
    case 'three_hops'
        numVars = 3; 
        lb = 0.25*ones(numVars, 1);
        ub = 0.75*ones(numVars, 1);
        options = optimoptions('ga','FunctionTolerance', 1e-2, ...
                                    'MaxGenerations', 10, ...
                                    'Display', 'iter'); 
        [x, f] = ga(@(x) objective(x,problem),numVars,[],[],[],[], ...
                                   lb,ub,[],options);                 
end

% ANALYZE SOLUTION
% Construct the actuator controls from solution 'x' and interpolate. See 
% CONSTRUCTCONTROLS below for details.
[deviceControl, muscleExcitation] = constructControls(x, problem);
time = 0:0.01:5;
deviceControlInterp    = interp1(deviceControl(1,:), ... 
                                 deviceControl(2,:), time);
muscleExcitationInterp = interp1(muscleExcitation(1,:), ... 
                                 muscleExcitation(2,:), time);
                             
% Plot optimized controls.
figure(1);
plot(time, deviceControlInterp, 'b-', 'linewidth', 2)
hold on
plot(time, muscleExcitationInterp, 'r-', 'linewidth', 2)
xlabel('time (s)')
ylabel('control value')

% Evaluate optimized solution. See BUILDHOPPERFROMSOLUTION and OBJECTIVE 
% below for details.
hopper = buildHopperFromSolution(x, problem);
[~, heightStruct] = EvaluateHopper(hopper, true, true);

% Plot jump height.
figure(2);
plot(heightStruct.time, heightStruct.height, 'b-', 'linewidth', 2)
xlabel('time (s)')
ylabel('height')

end

function f = objective(x, problem)
% OBJECTIVE
%   Simulate the hopper with the current controls and report hop height.

% See BUILDHOPPERFROMSOLUTION below for details.
hopper = buildHopperFromSolution(x, problem);

% Pass the hopper model to create a forward simulation with the current 
% guess for the actuator controls. This is the same function called after a
% hopper model is generated with BuildInteractiveHopperSolution from a 
% solution created in the InteractiveHopper GUI.
[peakHeight, ~] = EvaluateHopper(hopper, false, true);

% Negate peak height for minimization.
f = -peakHeight;

end

function hopper = buildHopperFromSolution(x, problem)
% BUILDHOPPERFROMSOLUTION
%   Build the hopper for the current iterate. The optimization variables 
%   'x' determine timing of the hop(s).

% Construct the actuator controls. See CONSTRUCTCONTROLS below for details.
[deviceControl, muscleExcitation] = constructControls(x, problem);
          
% Build the hopper with the passive device wrapped and the active device
% unwrapped (behind the knee). This is the same function called after a
% solution is created in the InteractiveHopper GUI. Here we use 'The Katie 
% Ledecky' muscle and set both the active device max force and passive 
% device stiffness to their maximum. 
hopper = BuildInteractiveHopperSolution(...
            'muscle', 'katieLedecky', ...
            'muscleExcitation', muscleExcitation, ...
            'addPassiveDevice', true, ...
            'passivePatellaWrap', true, ...
            'passiveParameter', 100, ...
            'addActiveDevice', true, ...
            'activePatellaWrap', false, ...
            'activeParameter', 100, ...
            'deviceControl', deviceControl, ...
            'isActivePropMyo', false);
end

function [deviceControl, muscleExcitation] = constructControls(x, problem)
% CONSTRUCTCONTROLS
%   Construct controls from the optimization optimization variables. Each
%   variable relates to either:
%       1) Turning OFF the active device (which releases the loaded spring)  
%          and turning ON the vastus muscle.
%       2) Turning ON the active device and turning OFF the vastus muscle
%          (to reload the spring).
%   
%   Each control is described by a 2xN array where the first row contains
%   time values, the second row contains control values, and N is the 
%   number of time points:
%
%                  control = [t1 t2 ... tN;
%                             u1 u2 ... uN];


% Anonymous functions to conveniently construct 'bang-bang' control signals 
% for both the active device and the muscle. 
% 
% Switch an actuator ON at time 't'. 
on  =@(t) [t  t+0.01; 
           0  1.0];
% Switch an actuator OFF at time 't'.
off =@(t) [t   t+0.01; 
           1.0 0];
       
% Construct the actuator controls by concatenating a sequence of 'on(t_i)'
% and 'off(t_i)' anonymous function calls at times t_i = [t1 t2 ... tN].
% Both problems start with by turning off the muscle and turning on the 
% active device at t = 0.
switch problem
    
    % The single variable in this problem determines when to simultaneously 
    % release a loaded passive device (turn OFF the active device) and 
    % activate the hopper vastus muscle (turn ON the muscle excitation).
    case 'one_hop'
        t1 = x;
        deviceControl    = [on(0)  off(t1)];
        muscleExcitation = [off(0) on(t1)];
        
    % To coordinate the sequence of hops in this problem, the spring must
    % be loaded and released (with the activated muscle) three times. The
    % first variable, x(1), is the wait time between when the spring is 
    % loaded and released for each hop. The x(1) value can be used for all 
    % three hops since we also optimize the wait time between each hop with 
    % the other two variables, x(2) and x(3). The active device is turned 
    % on at t = 0, so no variable is required to load the spring for the 
    % first hop.
    case 'three_hops'
        t1 = x(1);    % Release loaded spring and activate muscle (hop #1) 
        t2 = x(2)+t1; % Load spring after hop #1
        t3 = x(1)+t2; % Release loaded spring and activate muscle (hop #2)
        t4 = x(3)+t3; % Load spring after hop #2
        t5 = x(1)+t4; % Release loaded spring and activate muscle (hop #3)
        
        deviceControl    = [on(0)  off(t1) on(t2)  off(t3) on(t4)  off(t5)];
        muscleExcitation = [off(0) on(t1)  off(t2) on(t3)  off(t4) on(t5)];    
end

% For both muscle and device, impose a zero-order hold on the control value
% to the end of the jump cycle.
deviceControl    = [deviceControl(1,:) 5;
                    deviceControl(2,:) deviceControl(2,end)];
muscleExcitation = [muscleExcitation(1,:) 5;
                    muscleExcitation(2,:) muscleExcitation(2,end)];

end


