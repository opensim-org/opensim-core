% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
% Author(s): Daniel A. Jacobs
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

%   [x_dot, controlValues] = OpenSimPlantFunction(t, x, controlsFuncHandle, osimModel,
%   osimState) converts an OpenSimModel and an OpenSimState into a
%   function which can be passed as an input to a Matlab integrator, such as
%   ode45, or an optimization routine, such as fmin.
%
% Input:
%   t is the time at the current step
%   x is a Matlab column matrix of state values at the current step
%   controlsFuncHandle is a handle to a function which computes thecontrol
%   vector
%   osimModel is an org.opensim.Modeling.Model object
%   osimState is an org.opensim.Modeling.State object
%
% Output:
%   x_dot is a Matlab column matrix of the derivative of the state values
% -----------------------------------------------------------------------
function [x_dot, controlValues] = OpenSimPlantFunction(t, x, controlsFuncHandle, osimModel, ...
    osimState)
    % Error Checking
    if(~isa(osimModel, 'org.opensim.modeling.Model'))
        error('OpenSimPlantFunction:InvalidArgument', [...
            '\tError in OpenSimPlantFunction\n',...
            '\topensimModel is not type (org.opensim.modeling.Model).']);
    end
    if(~isa(osimState, 'org.opensim.modeling.State'))
        error('OpenSimPlantFunction:InvalidArgument', [...
            '\tError in OpenSimPlantFunction\n',...
            '\topensimState is not type (org.opensim.modeling.State).']);
    end
    if(size(x,2) ~= 1)
        error('OpenSimPlantFunction:InvalidArgument', [...
            '\tError in OpenSimPlantFunction\n',...
            '\tThe argument x is not a column matrix.']);
    end
    if(size(x,1) ~= osimState.getY().size())
        error('OpenSimPlantFunction:InvalidArgument', [...
            '\tError in OpenSimPlantFunction\n',...
            '\tThe argument x is not the same size as the state vector.',...
            'It should have %d rows.'], osimState.getY().size());
    end
%     if(~isa(controlsFunc, 'function_handle'))
%        error('OpenSimPlantFunction:InvalidArgument', [...
%             '\tError in OpenSimPlantFunction\n',...
%             '\tcontrolsFunc is not a valid function handle.']);
%     end

    % Check size of controls

    % Update state with current values
    osimState.setTime(t);
    numVar = osimState.getNY();
    for i = 0:1:numVar-1
        osimState.updY().set(i, x(i+1,1));
    end

    % Update the state velocity calculations
    osimModel.computeStateVariableDerivatives(osimState);

    % Update model with control values
    if(~isempty(controlsFuncHandle))
       controlVector = controlsFuncHandle(osimModel,osimState);
       osimModel.setControls(osimState, controlVector);
       for i = 1:osimModel.getNumControls()
           controlValues(1) = controlVector.get(i-1);
       end
    end

    % Update the derivative calculations in the State Variable
    osimModel.computeStateVariableDerivatives(osimState);

    x_dot = zeros(numVar,1);
    % Set output variable to new state
    for i = 0:1:numVar-1
        x_dot(i+1,1) = osimState.getYDot().get(i);
    end
end
