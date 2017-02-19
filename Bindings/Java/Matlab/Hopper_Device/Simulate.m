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
function Simulate(modelToCopy, state, visualize)
% Simulate an OpenSim model from an initial state. The provided state is
% updated to be the state at the end of the simulation.

import org.opensim.modeling.*;

% We make a copy of the model to avoid a dangling connection to the
% simbody-visualizer after it has been shut down.
model = modelToCopy.clone();
if visualize
    model.setUseVisualizer(true);
end
model.initSystem();

% Save this so that we can restart simulation from the given state.
% We use the copy constructor to perform a deep copy.
initState = State(state);

if visualize
    sviz = model.updVisualizer().updSimbodyVisualizer();
    sviz.setShowSimTime(true);
    % Show "ground and sky" background instead of just a black background.
    sviz.setBackgroundTypeByInt(1);
    % When model is deleted, the simbody-visualizer shuts down.
    % This is done in an attempt to avoid crashing MATLAB.
    sviz.setShutdownWhenDestructed(true);

    % Show help text in the visualization window.
    help = DecorativeText('Press any key to start a new simulation; ESC to quit.');
    help.setIsScreenText(true);
    sviz.addDecoration(0, Transform(Vec3(0, 0, 0)), help);

    model.getVisualizer().show(initState);

    % Wait for the user to hit a key before starting the simulation.
    silo = model.updVisualizer().updInputSilo();
end

while true
    if visualize
        % Ignore any previous key presses.
        silo.clear();
        % Get the next key press.
        key = silo.waitForKeyHitKeyOnly();
        if key == 27
            return;
        end
    end

    % TODO more intelligent checking for reporters in the Model.
    rep = TableReporter.safeDownCast(model.updComponent('hopper_results'));
    rep.clearTable();

    % Simulate.
    state = State(initState);
    manager = Manager(model);
    manager.integrate(state, 5.0);

    % If there is no visualizer, only simulate once.
    if ~visualize
        return;
    end
end

% This should cause the visualizer to shut down.
model.delete();

end
