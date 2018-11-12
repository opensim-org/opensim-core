% -------------------------------------------------------------------------- %
% OpenSim Muscollo: exampleMarkerTracking10DOF.m                             %
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

import org.opensim.modeling.*

% Create the 10-DOF skeletal model.
% =================================

% Load the base model from file.
model = Model('subject01.osim');

% Add coordinate actuators for each DOF in the model using a convenience
% function. See below for details.
addCoordinateActuator(model, 'lumbar_extension', 500);
addCoordinateActuator(model, 'pelvis_tilt', 500);
addCoordinateActuator(model, 'pelvis_tx', 1000);
addCoordinateActuator(model, 'pelvis_ty', 2500);
addCoordinateActuator(model, 'hip_flexion_r', 100);
addCoordinateActuator(model, 'knee_angle_r', 100);
addCoordinateActuator(model, 'ankle_angle_r', 100);
addCoordinateActuator(model, 'hip_flexion_l', 100);
addCoordinateActuator(model, 'knee_angle_l', 100);
addCoordinateActuator(model, 'ankle_angle_l', 100);

% Create MucoTool.
% ================
muco = MucoTool();
muco.setName('marker_tracking_10dof');

% Define the optimal control problem.
% ===================================
problem = muco.updProblem();

% Model (dynamics).
% -----------------
problem.setModel(model);

% Bounds.
% -------
% All bounds for state variables and model actuators are set based on model
% defaults or, for the coordinate speeds, defaults set by the problem. 
% Therefore, only the time bounds need to be set here.
problem.setTimeBounds(0, 1.25);

% Cost.
% -----
% Create a marker tracking cost term. This is the squared difference
% between the model markers and the experimental markers, integrated over
% the phase.
tracking = MucoMarkerTrackingCost();
tracking.setName('marker_tracking');

% First, obtain the experimental marker trajectories from the sample TRC
% file.
markerTraj = TRCFileAdapter().read('marker_trajectories.trc');

% Next, create a set of marker weights to define the relative importance for
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
markersRef = MarkersReference(markerTraj, markerWeights);
tracking.setMarkersReference(markersRef);

% This setting gives the marker tracking cost permission to ignore data in
% the markers reference for markers that don't exist in the model. For
% example, there are anatomical markers in the TRC file that got carried 
% over to markers reference that we'd like to ignore, so this flag is 
% enabled.
tracking.setAllowUnusedReferences(true);

% Add the tracking cost to the problem.
problem.addCost(tracking);

effort = MucoControlCost();
effort.set_weight(0.1);
problem.addCost(effort);

% Configure the solver.
% =====================
solver = muco.initSolver();
solver.set_num_mesh_points(30);
solver.set_optim_hessian_approximation('exact');
solver.setGuess('bounds');

% Now that we've finished setting up the tool, print it to a file.
muco.print('marker_tracking_10dof.omuco');

% Solve the problem.
% ==================
solution = muco.solve();
solution.write('marker_tracking_10dof_solution.sto');

% Visualize.
% ==========
if ~strcmp(getenv('OPENSIM_USE_VISUALIZER'), '0')
    muco.visualize(solution);
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
    plot(time, states(:,i))
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
    plot(time, controls(:,i))
    xlabel('time (s)')
    title(char(controlNames.get(i-1)))
end

end

function addCoordinateActuator(model, coordName, optimalForce)

import org.opensim.modeling.*

coordSet = model.updCoordinateSet();

actu = CoordinateActuator();
actu.setName(['tau_' coordName]);
actu.setCoordinate(coordSet.get(coordName));
actu.setOptimalForce(optimalForce);
actu.setMinControl(-1);
actu.setMaxControl(1);
model.addComponent(actu);

end
