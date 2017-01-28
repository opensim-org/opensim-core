function Simulate(model, state)
% Simulate an OpenSim model from an initial state. The provided state is
% updated to be the state at the end of the simulation.
% TODO license

import org.opensim.modeling.*;

sviz = model.updVisualizer().updSimbodyVisualizer();
% Show "ground and sky" background instead of just a black background.
sviz.setBackgroundTypeByInt(1);


% Show help text in the visualization window.
help = DecorativeText('Press any key to start a new simulation; ESC to quit.');
help.setIsScreenText(true);
sviz.addDecoration(0, Transform(Vec3(0, 0, 0)), help);

model.getVisualizer().show(state);

% Wait for the user to hit a key before starting the simulation.
silo = model.updVisualizer().updInputSilo();

% Save this so that we can restart simulation from the given state.
% We use the copy constructor to perform a deep copy.
initState = State(state);

while true
    % Ignore any previous key presses.
    silo.clear();
    % Get the next key press.
    key = silo.waitForKeyHitKeyOnly();
    if key == 27
        sviz.shutdown();
        return;
    end
    state = State(initState);
    manager = Manager(model);
    manager.setInitialTime(0.0);
    manager.setFinalTime(5.0);
    manager.integrate(state);
end

end
