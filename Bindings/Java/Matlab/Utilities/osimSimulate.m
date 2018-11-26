function osimSimulate(model, stateOrVisualize, finalTime)
% Simulate an OpenSim model.
%
% The simulation can be run with or without visualization (see stateOrVisualize
% for more information). If visualizing (with the simbody-visualizer), you will
% be able run the simulation multiple times.
% Otherwise, the simulation is run once without visualization.
%
% Use Ctrl-C in the command window to halt the simulation.
%
%
% Parameters
% ----------
% model: The OpenSim Model to simulate.
% stateOrVisualize:
%    State mode: The second parameter is the SimTK State to use as the initial
%        state for the simulation. In this case, you must have already called
%        initSystem() on the model. You can indicate if the visualizer should
%        be used via model.setUseVisualizer() (before calling initSystem()).
%        The provided state is updated to be the state at the end of the
%        simulation.
%    Visualize mode: The second parameter is a boolean indicating whether or
%        not the visualizer should be used. In this case, you need not call
%        initSystem() yourself. This function will call initSystem()
%        internally, and the simulation will start from the default state. In
%        this mode, this parameter is overridden by the OPENSIM_USE_VISUALIZER
%        environment variable, if set.
% finalTime: The final time for the simulation.

%-----------------------------------------------------------------------%
% The OpenSim API is a toolkit for musculoskeletal modeling and         %
% simulation. See http://opensim.stanford.edu and the NOTICE file       %
% for more information. OpenSim is developed at Stanford University     %
% and supported by the US National Institutes of Health (U54 GM072970,  %
% R24 HD065690) and by DARPA through the Warrior Web program.           %
%                                                                       %
% Copyright (c) 2017 Stanford University and the Authors                %
% Author(s): Thomas Uchida, Chris Dembia, Carmichael Ong, Nick Bianco,  %
%            Shrinidhi K. Lakshmikanth, Ajay Seth, James Dunne          %
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

import org.opensim.modeling.*;

stateMode = false;
if isa(stateOrVisualize, 'org.opensim.modeling.State')
    stateMode = true;
end

if stateMode
    visualize = model.getUseVisualizer();
    state = stateOrVisualize;
else
    visualize = stateOrVisualize;
    % This env. var. is used to turn off the visualizer during automated tests.
    if getenv('OPENSIM_USE_VISUALIZER') == '1'
        visualize = true;
    elseif getenv('OPENSIM_USE_VISUALIZER') == '0'
        visualize = false;
    end
    
    if visualize
        model.setUseVisualizer(true);
    end
    state = model.initSystem();
end

% Save this so that we can restart simulation from the given state.
% We use the copy constructor to perform a deep copy.
initState = State(state);

if visualize
    sviz = model.updVisualizer().updSimbodyVisualizer();
    sviz.setShowSimTime(true);
    % Show "ground and sky" background instead of just a black background.
    sviz.setBackgroundTypeByInt(1);

    % Show help text in the visualization window.
    help = DecorativeText('Press any key to start a new simulation; ESC to quit.');
    help.setIsScreenText(true);
    sviz.addDecoration(0, Transform(Vec3(0, 0, 0)), help);

    model.getVisualizer().show(initState);

    % Wait for the user to hit a key before starting the simulation.
    silo = model.updVisualizer().updInputSilo();
end

simulatedAtLeastOnce = false;
while true
    if visualize
        % Ignore any previous key presses.
        silo.clear();
        % Get the next key press.
        while ~silo.isAnyUserInput()
            pause(0.01);
        end
        % The alternative `waitForKeyHit()` is not ideal for MATLAB, as MATLAB
        % is not able to interrupt native functions, and `waitForKeyHit()` will
        % hang if the simbody-visualizer is killed.
        key = silo.takeKeyHitKeyOnly();
        % Key 27 is ESC; see the SimTK::Visualizer::InputListener::KeyCode enum.
        if key == 27
            sviz.shutdown();
            if ~simulatedAtLeastOnce
                error('User exited visualizer without running any simulations.')
            end
            return;
        end
    end

    % Clear the table for all TableReporters. Note: this does not handle
    % TableReporters for class Vector, etc.
    compList = model.getComponentsList();
    compIter = compList.begin();
    while ~compIter.equals(compList.end())
        if ~isempty(strfind(compIter.getConcreteClassName(), ...
                    'TableReporter__double_'))
            comp = model.getComponent(compIter.getAbsolutePathString());
            reporter = TableReporter.safeDownCast(comp);
            reporter.clearTable();
        elseif ~isempty(strfind(compIter.getConcreteClassName(), ...
                    'TableReporter__Vec3_'))
            comp = model.getComponent(compIter.getAbsolutePathString());
            reporterVec3 = TableReporterVec3.safeDownCast(comp);
            reporterVec3.clearTable();  
        end
        compIter.next();
    end

    % Simulate.
    thisState = State(initState);
    manager = Manager(model);
    manager.initialize(thisState);
    state = manager.integrate(finalTime);
    simulatedAtLeastOnce = true;

    % If there is no visualizer, only simulate once.
    if ~visualize
        return;
    end
end

end
