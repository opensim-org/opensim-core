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

% Build and simulate a single-legged hopping mechanism.

import org.opensim.modeling.*;

% This script defines the 'hopper' variable.
hopper = BuildHopperModel();
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
