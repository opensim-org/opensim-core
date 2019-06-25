% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleMocoTrack.m                                           %
% -------------------------------------------------------------------------- %
% Copyright (c) 2019 Stanford University and the Authors                     %
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

% This example features two different tracking problems solved using the
% MocoTrack tool. The first demonstrates the basic usage of the tool interface
% to solve a muscle-driven state tracking problem. The second problem shows
% how to customize a torque-driven marker tracking problem using more advanced
% features of the tool interface.

function exampleMocoTrack()

% Solve the muscle-driven state tracking problem.
muscleDrivenStateTracking();

% Solve the torque-driven marker tracking problem.
torqueDrivenMarkerTracking();
end

function muscleDrivenStateTracking()

import org.opensim.modeling.*;

% Create and name an instance of the MocoTrack tool.
track = MocoTrack();
track.setName("muscle_driven_state_tracking");

% Construct a ModelProcessor and set it on the tool. Here, external ground
% reaction forces are added in lieu of a foot-ground contact model and
% reserve actuators are added to supplement muscle forces. The default
% muscles in the model are replaced with optimization-friendly
% DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
% parameters.
modelProcessor = ModelProcessor("subject_walk_armless.osim");
modelProcessor.append(ModOpAddExternalLoads("grf_walk.xml"));
modelProcessor.append(ModOpAddReserves(5));
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
modelProcessor.append(ModOpIgnorePassiveFiberForces());
modelProcessor.append(ModOpScaleMaxIsometricForce(2));
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidth(1.5));
track.setModel(modelProcessor);

% Construct a TableProcessor of filtered coordinate value data from an
% OpenSim InverseKinematics solution.
tableProcessor = TableProcessor("coordinates.sto");
tableProcessor.append(TabOpLowPasFilter(6));
track.setStatesReference(tableProcessor);

% This setting allow extra data columns contained in the states reference
% that don't correspond to model coordinates.
track.set_allow_unused_references(true);

% Since there is only coordinate position data the states references, this
% setting is enable to fill in the missing coordinate speed data using
% the derivative of splined position data.
track.set_track_reference_position_derivatives(true);

% Initial time, final time, and mesh interval.
track.set_initial_time(0.81);
track.set_final_time(1.65);
track.set_mesh_interval(0.05);
track.set_control_effort_weight(0.1);

% Solve! The boolean argument indicates to visualize the solution.
solution = track.solve(true);

end

function torqueDrivenMarkerTracking()

% Create and name an instance of the MocoTrack tool.
track = MocoTrack();
track.setName("torque_driven_marker_tracking");

% Construct a ModelProcessor adding the external ground reaction forces as
% in the previous problem. Remove the muscles in the model and add reserve
% actuators which now act as the primary actuators in the system.
modelProcessor = ModelProcessor("subject_walk_armless.osim");
modelProcessor.append(ModOpAddExternalLoads("grf_walk.xml"));
modelProcessor.append(ModOpRemoveMuscles());
modelProcessor.append(ModOpAddReserves(250));
track.setModel(modelProcessor);

% Use this convenience function to set the MocoTrack markers reference
% directly from a TRC file. By default, the markers data is filtered at
% 6 Hz and if in millimeters, converted to meters. Also, allow extra marker
% data columns as in the previous problem.
track.setMarkersReferenceFromTRC("marker_trajectories.trc");
track.set_allow_unused_references(true);

% Increase the tracking weights for markers in the data set placed on bony
% landmarks compared to markers located on soft tissue.
markerWeights = MocoWeightSet();
markerWeights.cloneAndAppend({"R.ASIS", 20});
markerWeights.cloneAndAppend({"L.ASIS", 20});
markerWeights.cloneAndAppend({"R.PSIS", 20});
markerWeights.cloneAndAppend({"L.PSIS", 20});
markerWeights.cloneAndAppend({"R.Knee", 10});
markerWeights.cloneAndAppend({"R.Ankle", 10});
markerWeights.cloneAndAppend({"R.Heel", 10});
markerWeights.cloneAndAppend({"R.MT5", 5});
markerWeights.cloneAndAppend({"R.Toe", 2});
markerWeights.cloneAndAppend({"L.Knee", 10});
markerWeights.cloneAndAppend({"L.Ankle", 10});
markerWeights.cloneAndAppend({"L.Heel", 10});
markerWeights.cloneAndAppend({"L.MT5", 5});
markerWeights.cloneAndAppend({"L.Toe", 2});
track.set_markers_weight_set(markerWeights);

% Initial time, final time, and mesh interval.
track.set_initial_time(0.81);
track.set_final_time(1.65);
track.set_mesh_interval(0.05);

% Instead of calling solve(), call initialize() to receive a pre-configured
% MocoStudy object based on the settings above. Use this to customize the
% problem beyond the MocoTrack interface.
moco = track.initialize();

% Get a reference to the MocoControlCost that is added to every MocoTrack
% problem by default.
problem = moco.updProblem();
effort = MocoControlCost.safeDownCast(problem.updCost("control_effort"));

% Put a large weight on the pelvis CoordinateActuators, which act as the
% residual, or 'hand-of-god', forces which we would like to keep as small
% as possible.
% model = modelProcessor.process();
% for (const auto& coordAct : model.getComponentList<CoordinateActuator>()) {
%      coordPath = coordAct.getAbsolutePathString();
%     if (coordPath.find("pelvis") != std::string::npos) {
%         effort.setWeight(coordPath, 1000);
%     end
% end
% 
% % Solve and visualize.
% MocoSolution solution = moco.solve();
% moco.visualize(solution);

end
