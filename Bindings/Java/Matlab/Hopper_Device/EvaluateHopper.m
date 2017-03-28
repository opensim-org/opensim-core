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

% Perform a hop with the provided model and evaluate the performance of the
% hop.

function [score, peakHeight, metabolicCost, deviceEnergyConsumption] = ...
        EvaluateHopper(hopper)

import org.opensim.modeling.*;

% TODO make sure this isn't causing a memory leak.
hopperCopy = hopper.clone();
hopperCopy.finalizeFromProperties();

hopperCopy.setUseVisualizer(false);

% Set up reporters.
% -----------------
heightRep = TableReporterVec3();
heightRep.setName('com_height_reporter');
% Reducing the reporting interval from 0.10 vs 0.05 only increases runtime of
% this function by about 1.5%.
heightRep.set_report_time_interval(0.05);
heightRep.addToReport(hopperCopy.getOutput('com_position'), 'com_position');
hopperCopy.addComponent(heightRep);

metRep = TableReporterVector();
metRep.setName('metabolics_reporter');
metRep.set_report_time_interval(0.05);
metRep.addToReport(...
        hopperCopy.getComponent('Umberger').getOutput('probe_outputs'), ...
        'metabolic_rate_total');
hopperCopy.addComponent(metRep);

powerRep = TableReporter();
powerRep.setName('device_power_reporter');
powerRep.set_report_time_interval(0.05);

% TODO handle multiple devices.
% TODO it is not yet possible to loop through components of a given type in
% MATLAB.
if hopperCopy.hasComponent('device_active')
    powerRep.addToReport(...
            hopperCopy.getComponent('device_active').getOutput('power'), ...
            'device_power');
end
hopperCopy.addComponent(powerRep);

% Simulate.
% ---------
state = hopperCopy.initSystem();
% The last argument determines if the simbody-visualizer should be used.
Simulate(hopperCopy, state, false);

% Process reporter tables.
% ------------------------
heightTable = heightRep.getTable();
height = opensimTimeSeriesTableToMatlab(heightTable);
[peakHeight, maxHeightIdx] = max(height.com_position(:, 2));
fprintf('Peak mass center height: %f meters (at time %f seconds)\n', ...
        peakHeight, height.time(maxHeightIdx));

metTable = metRep.getTable();
met = opensimTimeSeriesTableToMatlab(metTable);
% opensimTimeSeriesTableToMatlab thinks this field has 3 columns; take only
% the first column.
metabolicCost = trapz(met.time, met.metabolic_rate_total_0_(:, 1));
fprintf('Metabolic cost: %f Joules\n', metabolicCost);

powerTable = powerRep.getTable();
power = opensimTimeSeriesTableToMatlab(powerTable);
% opensimTimeSeriesTableToMatlab thinks this field has 3 columns; take only
% the first column.
deviceEnergyConsumption = trapz(power.time, power.device_power(:, 1));
fprintf('Device energy consumption: %f Joules\n', deviceEnergyConsumption);

score = peakHeight / (metabolicCost + deviceEnergyConsumption);
fprintf('Score: %f\n', score);

end
