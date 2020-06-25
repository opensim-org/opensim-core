% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleSynergies.m                                           %
% -------------------------------------------------------------------------- %
% Copyright (c) 2020 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Christopher Dembia                                              %
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

import org.opensim.modeling.*;

% Create Model.
% =============

model = ModelFactory.createPlanarPointMass();
model.finalizeConnections();
model.set_gravity(Vec3(0));
model.printSubcomponentInfo();

controller = SynergyController();
controller.addActuator(model.getActuators().get('force_x'));
controller.addActuator(model.getActuators().get('force_y'));
controller.setSynergyWeights('/forceset/force_x', Vector.createFromMat([1.0]));
controller.setSynergyWeights('/forceset/force_y', Vector.createFromMat([2.0]));

signal = SignalGenerator();
signal.set_function(Constant(1.5));
concat = Concatenator();
concat.connectInput_inputs(signal.getOutput('signal'));

controller.connectInput_synergy_controls(concat.getOutput('output'));
controller.addComponent(signal);
controller.addComponent(concat);
model.addController(controller);

% Conduct forward simulation.
% ===========================
model.setUseVisualizer(true);
state = model.initSystem();
finalTime = 0.5;
finalState = opensimSimulation.simulate(model, state, finalTime);

disp(finalState.getQ());

% Create MocoStudy.
% ================

% TODO
