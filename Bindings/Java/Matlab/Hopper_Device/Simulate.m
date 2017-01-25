function Simulate(model, state)
% Simulate an OpenSim model from an initial state. The provided state is
% updated to be the state at the end of the simulation.
% TODO license

import org.opensim.modeling.*;
model.getVisualizer().show(state);
input('Hit ENTER to start the simulation.');

manager = Manager(model);
manager.setInitialTime(0.0);
manager.setFinalTime(5.0);
manager.integrate(state);

end
