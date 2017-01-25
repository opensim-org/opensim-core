% TODO license.

import org.opensim.modeling.*;

% This defines the 'hopper` variable.
BuildHopperModel;
% Show a visualization window when simulating.
hopper.setUseVisualizer(true);

%ShowSubcomponentInfo(hopper);
%ShowAllOutputs(hopper.getComponent('/Dennis/thigh'), false);

%AddConsoleReporterToHopper(hopper);
sHop = hopper.initSystem();
Simulate(hopper, sHop);

%hopper.setUseVisualizer(true);
%
%state = hopper.initSystem();
%hopper.getVisualizer().show(state)
%
%manager = Manager(hopper);
%manager.setFinalTime(10.0);
%manager.integrate(state);
%
%hopper.print('Hopper.osim');
