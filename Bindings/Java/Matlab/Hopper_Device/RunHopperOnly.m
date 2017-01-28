% TODO license.

% Build and simulate a single-legged hopping mechanism.

import org.opensim.modeling.*;

% This script defines the 'hopper' variable.
BuildHopperModel;
% Show a visualization window when simulating.
hopper.setUseVisualizer(true);
%hopper.print('Hopper.osim'); TODO walk through the OSIM file?

hopper.dumpSubcomponentInfo();
hopper.getComponent('/Dennis/thigh').dumpOutputInfo();

% Create a new ConsoleReporter. Set its name and reporting interval.
reporter = ConsoleReporter();
reporter.setName('hopper_results');
reporter.set_report_time_interval(0.2); % seconds

% Connect outputs from the hopper to the reporter's inputs. Try reporting the
% hopper's height, the vastus muscle's activation, the knee angle, and any
% other variables of interest.
% The last argument is an alias that is used for this quantity during reporting.
reporter.addToReport(...
    hopper.getComponent('/Dennis/slider/yCoord').getOutput('value'), 'height');
reporter.addToReport(...
    hopper.getComponent('/Dennis/vastus').getOutput('activation'));
reporter.addToReport(...
    hopper.getComponent('/Dennis/knee/kneeFlexion').getOutput('value'), ...
    'knee_angle');

hopper.addComponent(reporter);
sHop = hopper.initSystem();
Simulate(hopper, sHop);
