% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
% Author(s): Daniel A. Jacobs, Chris Dembia
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

%   outVector = OpenSimPlantControlsFunction(osimModel, osimState)
%   This function computes a control vector for the model's
%   actuators.  The current code is for use with the script
%   DesignMainStarterWithControls.m
%
% Input:
%   osimModel is an org.opensim.Modeling.Model object
%   osimState is an org.opensim.Modeling.State object
%
% Output:
%   outVector is an org.opensim.Modeling.Vector of the control values
% -----------------------------------------------------------------------
function modelControls = OpenSimPlantControlsFunction(osimModel, osimState)
    % Load Library
    import org.opensim.modeling.*;

    % Check Size
    if(osimModel.getNumControls() < 1)
       error('OpenSimPlantControlsFunction:InvalidControls', ...
           'This model has no controls.');
    end

    % Get a reference to current model controls
    modelControls = osimModel.updControls(osimState);

    % Initialize a vector for the actuator controls
    % Most actuators have a single control.  For example, muscle have a
    % signal control value (excitation);
    actControls = Vector(1, 0.0);

    % Calculate the controls based on any proprty of the model or state
    LKnee_rz = osimModel.getCoordinateSet().get('LKnee_rz').getValue(osimState);
    LKnee_rz_u = osimModel.getCoordinateSet().get('LKnee_rz').getSpeedValue(osimState);

    % Position Control to slightly flexed Knee
    wn = 5.0;
    kp = wn^2;
    kv = 2*wn;
    LKnee_rz_des = -90*pi/180;
    val = -kv * LKnee_rz_u - kp * (LKnee_rz - LKnee_rz_des);

    % Set Actuator Controls
    actControls.set(0, val);

    % Update modelControls with the new values
    osimModel.updActuators().get('coordAct_LK').addInControls(actControls, modelControls);

 end
