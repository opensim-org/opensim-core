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

% This function builds the hopper model; no need to edit it.
hopper = BuildHopper();

% TODO: Discover the subcomponents in the hopper, and the outputs available for
%       reporting. Identify the outputs for the hopper's height and muscle
%       activation.
% [Step 1, Task A]
% ANSWER{
hopper.printSubcomponentInfo();
hopper.getComponent('slider').printOutputInfo();
hopper.getComponent('vastus').printOutputInfo();
% }

% TODO: Create a TableReporter, give it a name, and set its reporting interval
%       to 0.2 seconds. Wire the hopper's height and muscle activation
%       outputs to the reporter. Then add the reporter to the hopper.
%       Adding an output to the reporter looks like the following; the alias
%       becomes the column label in the table. 
%         reporter.addToReport(...
%           hopper.getComponent('<comp-path>').getOutput('<output-name>'), ...
%           '<alias>');
% [Step 1, Task B]
% ANSWER{
reporter = TableReporter();
reporter.setName('hopper_results');
reporter.set_report_time_interval(0.2); % seconds
reporter.addToReport(...
    hopper.getComponent('slider/yCoord').getOutput('value'), 'height');
reporter.addToReport(...
    hopper.getComponent('vastus').getOutput('activation'));
hopper.addComponent(reporter);
% }

sHop = hopper.initSystem();
% The last argument determines if the simbody-visualizer should be used.
Simulate(hopper, sHop, true);

% TODO: Display the TableReporter's data, and save it to a file.
% [Step 1, Task C]
% ANSWER{
table = reporter.getTable();
disp(table.toString());
csv = CSVFileAdapter();
csv.write(table, 'hopper_results.csv');
% }
  
% TODO: Convert the TableReporter's Table to a MATLAB struct and plot the
%       the hopper's height over the motion.
% [Step 1, Task D]
% ANSWER{
results = opensimTimeSeriesTableToMatlab(table);
fieldnames(results)
figure;
plot(results.time, results.height);
xlabel('time');
ylabel('height');
% }
