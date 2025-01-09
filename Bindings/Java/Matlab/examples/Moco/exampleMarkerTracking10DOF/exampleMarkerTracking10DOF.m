% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleMarkerTracking10DOF.m                                 %
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

function exampleMarkerTracking10DOF()

import org.opensim.modeling.*;

% Create the 10-DOF skeletal model.
% =================================
% Load the base model from file.
scriptdir = fileparts(mfilename('fullpath'));
model = Model(fullfile(scriptdir, 'subject01.osim'));

% Add CoordinateActuators for each DOF in the model using a 
% convenience function. See below for details.
addCoordinateActuator(model, 'lumbar_extension', 100);
addCoordinateActuator(model, 'pelvis_tilt', 100);
addCoordinateActuator(model, 'pelvis_tx', 200);
addCoordinateActuator(model, 'pelvis_ty', 1500);
addCoordinateActuator(model, 'hip_flexion_r', 50);
addCoordinateActuator(model, 'knee_angle_r', 20);
addCoordinateActuator(model, 'ankle_angle_r', 5);
addCoordinateActuator(model, 'hip_flexion_l', 50);
addCoordinateActuator(model, 'knee_angle_l', 20);
addCoordinateActuator(model, 'ankle_angle_l', 5);

% Create MocoStudy.
% ================
study = MocoStudy();
study.setName('marker_tracking_10dof');

% Define the optimal control problem.
% ===================================
problem = study.updProblem();

% Model (dynamics).
% -----------------
problem.setModel(model);

% Bounds.
% -------
% All bounds for coordinate position values and actuator controls are set 
% based on model default ranges. The coordinate speeds are set to [-50, 50]
% by the problem as a default. All of these values can be modified on the 
% problem itself. For example,
%   problem.setStateInfo('joint/coord/value', [-10, 10]);
%   problem.setStateInfo('joint/coord/speed', [-30, 30]);
%   problem.setControlInfo('actuator', [0, 1]);
% Only the time bounds need to be set here.
problem.setTimeBounds(0, 1.25);

% Cost.
% -----
% Create a marker tracking cost term. This term will compute the squared 
% difference between the model markers and the experimental markers, 
% integrated over the phase.
markerTrackingCost = MocoMarkerTrackingGoal();
markerTrackingCost.setName('marker_tracking');

% Create a set of marker weights to define the relative importance for
% tracking individual markers.
markerWeights = SetMarkerWeights();
markerWeights.cloneAndAppend(MarkerWeight('Top.Head', 3));
markerWeights.cloneAndAppend(MarkerWeight('R.ASIS', 3));
markerWeights.cloneAndAppend(MarkerWeight('L.ASIS', 3));
markerWeights.cloneAndAppend(MarkerWeight('V.Sacral', 3));
markerWeights.cloneAndAppend(MarkerWeight('R.Heel', 2));
markerWeights.cloneAndAppend(MarkerWeight('R.Toe.Tip', 2));
markerWeights.cloneAndAppend(MarkerWeight('L.Heel', 2));
markerWeights.cloneAndAppend(MarkerWeight('L.Toe.Tip',2));

% Create an OpenSim::MarkersReference from the experimental marker
% trajectories and the marker weights and pass it to the tracking cost. The
% cost term uses this marker reference to compute the weighted, squared
% marker tracking error internally.
markersRef = MarkersReference(fullfile(scriptdir, 'marker_trajectories.trc'), ...
    markerWeights);
markerTrackingCost.setMarkersReference(markersRef);

% This setting gives the marker tracking cost permission to ignore data in
% the markers reference for markers that don't exist in the model. For
% example, there are anatomical markers in the TRC file that got carried 
% over to markers reference that we'd like to ignore, so this flag is 
% enabled.
markerTrackingCost.setAllowUnusedReferences(true);

% Add the tracking cost to the problem.
problem.addGoal(markerTrackingCost);

% Add a low-weighted control effort cost to reduce oscillations in the 
% actuator controls.
controlCost = MocoControlGoal();
controlCost.setWeight(0.001);
problem.addGoal(controlCost);

% Configure the solver.
% =====================
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(20);
solver.set_optim_constraint_tolerance(1e-3);
solver.set_optim_convergence_tolerance(1e-3);

solver.setGuess('bounds');

% Now that we've finished setting up the tool, print it to a file.
study.print('marker_tracking_10dof.omoco');

% Solve the problem.
% ==================
solution = study.solve();
solution.write('marker_tracking_10dof_solution.sto');

% Visualize.
% ==========
% The following environment variable is set during automated testing.
if ~strcmp(getenv('OPENSIM_USE_VISUALIZER'), '0')
    study.visualize(solution);
end

% Plot states trajectory solution.
% --------------------------------
set(groot, 'defaultTextInterpreter', 'none')
figure;
time = solution.getTimeMat();
states = solution.getStatesTrajectoryMat();
stateNames = solution.getStateNames();
for i = 1:size(states ,2)
    subplot(4, 5, i)
    plot(time, states(:,i), 'linewidth', 2)
    xlabel('time (s)')
    title(char(stateNames.get(i-1)))
end

% Plot controls trajectory solution.
% ----------------------------------
figure;
controls = solution.getControlsTrajectoryMat();
controlNames = solution.getControlNames();
for i = 1:size(controls ,2)
    subplot(2, 5, i)
    plot(time, controls(:,i), 'linewidth', 2)
    xlabel('time (s)')
    title(char(controlNames.get(i-1)))
end

end

function addCoordinateActuator(model, coordName, optimalForce)

import org.opensim.modeling.*;

coordSet = model.updCoordinateSet();

actu = CoordinateActuator();
actu.setName(['tau_' coordName]);
actu.setCoordinate(coordSet.get(coordName));
actu.setOptimalForce(optimalForce);
% Set the min and max control defaults to automatically generate bounds for
% each actuator in the problem.
actu.setMinControl(-1);
actu.setMaxControl(1);
model.addComponent(actu);

end
