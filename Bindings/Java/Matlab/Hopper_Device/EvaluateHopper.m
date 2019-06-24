function [peakHeight, heightStruct] = EvaluateHopper(hopper, visualize, print)
% Simulate a hop with the provided hopper model and evaluate the performance of
% the hop.
%
% Parameters
% ----------
% model: The OpenSim hopper model to evaluate.
% visualize (bool): Use the simbody-visualizer to visualize the simulation?
% print (bool): Print peak height to the command window?
 
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

hopperCopy = hopper.clone();

hopperCopy.setUseVisualizer(false);

% Set up reporters.
% -----------------
heightRep = TableReporter();
heightRep.setName('height_reporter');
% Reducing the reporting interval from 0.10 vs 0.05 only increases runtime of
% this function by about 1.5%.
heightRep.set_report_time_interval(0.05);
yCoord = hopperCopy.getComponent('jointset/slider/yCoord');
heightRep.addToReport(yCoord.getOutput('value'), 'height');
hopperCopy.addComponent(heightRep);

% Simulate.
% ---------
% The second argument determines if the simbody-visualizer should be used.
% The third argument is the simulation duration.
osimSimulate(hopperCopy, visualize, 5.0);

% Process reporter tables.
% ------------------------
heightTable = heightRep.getTable();
heightStruct = osimTableToStruct(heightTable);
[peakHeight, maxHeightIdx] = max(heightStruct.height(:, 1));
if print 
    fprintf('Peak height: %f meters (at time %f seconds)\n', ...
        peakHeight, heightStruct.time(maxHeightIdx));
end

end
