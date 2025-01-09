% -----------------------------------------------------------------------
% The OpenSim API is a toolkit for musculoskeletal modeling and
% simulation. See http://opensim.stanford.edu and the NOTICE file
% for more information. OpenSim is developed at Stanford University
% and supported by the US National Institutes of Health (U54 GM072970,
% R24 HD065690) and by DARPA through the Warrior Web program.
%
% Copyright (c) 2005-2019 Stanford University and the Authors
% Author(s): Daniel A. Jacobs, Ajay Seth
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% http://www.apache.org/licenses/LICENSE-2.0.                       
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
% implied. See the License for the specific language governing
% permissions and limitations under the License.
% -----------------------------------------------------------------------

% OutputData = IntegrateOpenSimPlant(osimModel, controlsFuncHandle,...
%    timeSpan, integratorName, integratorOptions)
%
%   IntegrateOpenSimPlant is a function for integrating a
%   OpenSim model using one of Matlab's integrator routines.
%
% Input:
%   osimModel: An OpenSim Model object
%   controlsFuncHandle: an optional function handle which can be used to
%       calculate the controls applied to actuators at each time step.
%   timeSpan: A row matrix of time ranges for the integrator. This can be
%       timeRange = [timeInitial timeFinal] or [t1, t2, t3 ... timeFinal].
%   integratorName: A char array of the specific integrator to use
%   integratorOptions: a set of integrator options generated with odeset
%   (for defaults, pass an empty array).
%
% Output:
%   The output of this script is a Matlab structure named OutputData. The
%   format of this structure can be passed to PlotOpenSimFunction.m for
%   plotting.
%
%   The stucture fields are:
%       name: A char array identifier of the data
%       nRows: the number of rows of data in the data field
%       nColumns: the number of columns of data in the data field
%       labels: an array of char arrays of data names from the header file
%       data: a nRows by nColumnss matrix of data values
%
% Usage:
% outputDataStructure = IntegrateOpenSimPlant(osimModel, osimState, ...
% timeSpan, integratorName, integratorOptions);
% -----------------------------------------------------------------------
function OutputData = IntegrateOpenSimPlant(osimModel, controlsFuncHandle,...
    timeSpan, integratorName, integratorOptions)

    % Import Java libraries
    import org.opensim.modeling.*;

    if(~isa(osimModel, 'org.opensim.modeling.Model'))
        error('IntegrateOpenSimPlant:InvalidArgument', [ ...
            '\tError in IntegrateOpenSimPlant\n', ...
            '\tArgument osimModel is not an org.opensim.modeling.Model.']);
    end
    if(~isempty(controlsFuncHandle))
        if(~isa(controlsFuncHandle, 'function_handle'))
            controlsFuncHandle = [];
            display('controlsFuncHandle was not a valid function_handle');
            display('No controls will be used.');
        end
    end

    % Check to see if model state is initialized by checking size
    if(osimModel.getWorkingState().getNY() == 0)
       osimState = osimModel.initSystem();
    else
       osimState = osimModel.updWorkingState();
    end

    % Create the Initial State matrix from the Opensim state
    numVar = osimState.getY().size();
    InitStates = zeros(numVar,1);
    for i = 0:1:numVar-1
        InitStates(i+1,1) = osimState.getY().get(i);
    end

    if(osimModel.getUseVisualizer())
        % Create a anonymous handle to the OpenSim visualize function.
        visualizeHandle = @(t, x, flag) ...
            OpenSimVisualizeFunction(t, x, flag, osimModel, osimState, ...
            osimState.getNY());

        integratorOptions.OutputFcn = visualizeHandle;
        freq = 30; % frames/second;
        timeSpan = [timeSpan(1):1/freq:timeSpan(end)];
    end

    % Create a anonymous handle to the OpenSim plant function.  The
    % variables osimModel and osimState are held in the workspace and
    % passed as arguments
    plantHandle = @(t,x) OpenSimPlantFunction(t, x, controlsFuncHandle, osimModel, osimState);

    tic;
    % Integrate the system equations
    integratorFunc = str2func(integratorName);
    [T, Y] = integratorFunc(plantHandle, timeSpan, InitStates, ...
        integratorOptions);

    simTime = toc();
    sprintf('Simulation time = %d.', simTime);

    % Create Output Data structure
    OutputData = struct();
    OutputData.name = [char(osimModel.getName()), '_states'];
    OutputData.nRows = size(T, 1);
    OutputData.nColumns = size(T, 2) + size(Y, 2);
    OutputData.inDegrees = false;
    OutputData.labels = cell(1,OutputData.nColumns);
    OutputData.labels{1}= 'time';
    for j = 2:1:OutputData.nColumns
        OutputData.labels{j} = char(osimModel.getStateVariableNames().getitem(j-2));
    end
    OutputData.data = [T, Y];
end

function status = OpenSimVisualizeFunction(t, x, flag, opensimModel, ...
    opensimState, stateSize)
    status = 0;

    % Flag has three values, init, empty and done
    % Init and Done are for initialization and cleanup
    % Empty indicates a succesful step has been made
    if(isempty(flag))
        opensimState.setTime(t(1));
        for i = 0:1:stateSize-1
            opensimState.updY().set(i, x(i+1,1));
        end
        opensimModel.getVisualizer().show(opensimState);
    end
end
