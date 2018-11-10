% -------------------------------------------------------------------------- %
% OpenSim Muscollo: prescribeSolutionControlsToModel.m                       %
% -------------------------------------------------------------------------- %
% Copyright (c) 2017 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Nicholas Bianco                                                 %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the "License"); you may    %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0          %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an "AS IS" BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %

% Given a valid MucoSolution obtained from solving a MucoProblem and the 
% associated OpenSim model, return the model with a prescribed controller 
% appended that will compute the control values from the MucoSolution. This can
% be useful when computing state-dependent model quantities that require 
% realization to the Dynamics stage or later.
function model = prescribeSolutionControlsToModel(solution, model)

import org.opensim.modeling.*;

% Get an array of the actuator names in the model.
actuNames = ArrayStr();
for i = 0:model.getActuators().getSize()-1 
    actu = model.getActuators().get(i);
    actuNames.append(actu.getName());
end

% Create the prescribed controller.
controller = PrescribedController();
controller.setName('prescribed_controller');

% Get the time vector from the MucoSolution.
time = solution.getTimeMat();

% Create a control function for each actuator in the model and add it to the
% prescribed controller.
for i = 0:actuNames.getSize()-1 
   % Get a vector of the control values from the MucoSolution.
   control = solution.getControlMat(actuNames.get(i));
   
   % Create a function based on a spline of the control solution.
   controlFunction = GCVSpline();
   controlFunction.setDegree(5);
   for t = 1:length(time)
      controlFunction.addPoint(time(t), control(t)); 
   end
   
   % Add the model actuator to the controller.
   controller.addActuator(model.getActuators().get(actuNames.get(i)));
   
   % Associate the spline control function with the model actuator in the
   % controller.
   controller.prescribeControlForActuator(actuNames.get(i), controlFunction);
end

% Add the controller to the model.
model.addController(controller);

end