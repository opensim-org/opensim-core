% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleMocoTrack.m                                           %
% -------------------------------------------------------------------------- %
% Copyright (c) 2023 Stanford University and the Authors                     %
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

% This example features three different tracking problems solved using the
% MocoTrack tool. 
%  - The first problem demonstrates the basic usage of the tool interface
%    to solve a torque-driven marker tracking problem. 
%  - The second problem shows how to customize a muscle-driven state tracking 
%    problem using more advanced features of the tool interface.
%  - The third problem demonstrates how to solve a muscle-driven joint moment
%    tracking problem.
%
% This example also shows how to use the osimMocoTrajectoryReport.m utility to
% create a PDF report of plots of a solution.
%
% See the README.txt next to this file for more information.

function exampleMocoTrack()

% Solve the torque-driven marker tracking problem.
torqueDrivenMarkerTracking();

% Solve the muscle-driven state tracking problem.
muscleDrivenStateTracking();

% Solve the muscle-driven joint moment tracking problem.
muscleDrivenJointMomentTracking();

end

function torqueDrivenMarkerTracking()

import org.opensim.modeling.*;

% Create and name an instance of the MocoTrack tool.
track = MocoTrack();
track.setName("torque_driven_marker_tracking");

% Construct a ModelProcessor and add it to the tool. ModelProcessors
% accept a base model and allow you to easily modify the model by appending
% ModelOperators. Operations are performed in the order that they are
% appended to the model.
% Create the base Model by passing in the model file.
modelProcessor = ModelProcessor("subject_walk_scaled.osim");
% Replace the PinJoints representing the model's toes with WeldJoints.
jointsToWeld = StdVectorString();
jointsToWeld.add("mtp_r");
jointsToWeld.add("mtp_l");
modelProcessor.append(ModOpReplaceJointsWithWelds(jointsToWeld));
% Add ground reaction external loads in lieu of a ground-contact model.
modelProcessor.append(ModOpAddExternalLoads("grf_walk.xml"));
% Remove all the muscles in the model's ForceSet.
modelProcessor.append(ModOpRemoveMuscles());
% Add CoordinateActuators to the pelvis coordinates. 
modelProcessor.append(ModOpAddResiduals(250.0, 50.0, 1.0));
% Add CoordinateActuators to the remaining degrees-of-freedom. 
modelProcessor.append(ModOpAddReserves(250.0, 1.0));
track.setModel(modelProcessor);

% Use this convenience function to set the MocoTrack markers reference
% directly from a TRC file. By default, the markers data is filtered at
% 6 Hz and if in millimeters, converted to meters.
track.setMarkersReferenceFromTRC("markers_walk.trc");

% Increase the global marker tracking weight, which is the weight
% associated with the internal MocoMarkerTrackingGoal term.
track.set_markers_global_tracking_weight(10);

% Set the marker weights based on the IKTaskSet from the dataset.
ikTaskSet = IKTaskSet("ik_tasks_walk.xml")
track.setMarkerWeightsFromIKTaskSet(ikTaskSet)

% Initial time, final time, and mesh interval. The number of mesh points
% used to discretize the problem is computed internally using these values.
track.set_initial_time(0.48);
track.set_final_time(1.61);
track.set_mesh_interval(0.02);

% Solve! Use track.solve() to skip visualizing.
solution = track.solveAndVisualize();

solution.write('exampleMocoTrack_markertracking_solution.sto');

% Generate a PDF report containing plots of the variables in the solution.
% For details, see osimMocoTrajectoryReport.m in Moco's
% Resources/Code/Matlab/Utilities folder.
model = modelProcessor.process();
report = osimMocoTrajectoryReport(model, ...
            'exampleMocoTrack_markertracking_solution.sto');
reportFilepath = report.generate();
open(reportFilepath);

end

function muscleDrivenStateTracking()

import org.opensim.modeling.*;

% Create and name an instance of the MocoTrack tool. 
track = MocoTrack();
track.setName("muscle_driven_state_tracking");

% Construct a ModelProcessor and set it on the tool. The default
% muscles in the model are replaced with optimization-friendly
% DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
% parameters.
modelProcessor = ModelProcessor("subject_walk_scaled.osim");
jointsToWeld = StdVectorString();
jointsToWeld.add("mtp_r");
jointsToWeld.add("mtp_l");
modelProcessor.append(ModOpReplaceJointsWithWelds(jointsToWeld));
modelProcessor.append(ModOpAddExternalLoads("grf_walk.xml"));
% Add CoordinateActuators to the pelvis coordinates. 
modelProcessor.append(ModOpAddResiduals(250.0, 50.0, 1.0));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
% Use a function-based representation for the muscle paths. This is
% recommended to speed up convergence, but if you would like to use
% the original GeometryPath muscle wrapping instead, simply comment out
% this line. To learn how to create a set of function-based paths for
% your model, see the example 'examplePolynomialPathFitter.m'.
modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(...
        "subject_walk_scaled_FunctionBasedPathSet.xml"));
track.setModel(modelProcessor);

% Construct a TableProcessor of the coordinate data and pass it to the 
% tracking tool. TableProcessors can be used in the same way as
% ModelProcessors by appending TableOperators to modify the base table.
% A TableProcessor with no operators, as we have here, simply returns the
% base table.
track.setStatesReference(TableProcessor("coordinates.sto"));

% This setting allows extra data columns contained in the states
% reference that don't correspond to model coordinates.
track.set_allow_unused_references(true);

% Since there is only coordinate position data in the states references, this
% setting is enabled to fill in the missing coordinate speed data using
% the derivative of splined position data.
track.set_track_reference_position_derivatives(true);

% Initial time, final time, and mesh interval.
track.set_initial_time(0.48);
track.set_final_time(1.61);
track.set_mesh_interval(0.02);

% Instead of calling solve(), call initialize() to receive a pre-configured
% MocoStudy object based on the settings above. Use this to customize the
% problem beyond the MocoTrack interface.
study = track.initialize();

% Get a reference to the MocoControlGoal that is added to every MocoTrack
% problem by default.
problem = study.updProblem();
effort = MocoControlGoal.safeDownCast(problem.updGoal("control_effort"));
effort.setWeight(0.1);

% Put a large weight on the pelvis CoordinateActuators, which act as the
% residual, or 'hand-of-god', forces which we would like to keep as small
% as possible.
effort.setWeightForControlPattern('.*pelvis.*', 10);

% Constrain the states and controls to be periodic.
periodicityGoal = MocoPeriodicityGoal("periodicity");
model = modelProcessor.process();
model.initSystem();
for i = 0:model.getNumStateVariables()-1
    currentStateName = string(model.getStateVariableNames().getitem(i));
    if (~contains(currentStateName,'pelvis_tx/value'))
        symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
    end
end

forceSet = model.getForceSet();
for i = 0:forceSet.getSize()-1
    forcePath = forceSet.get(i).getAbsolutePathString();
    symmetryGoal.addControlPair(MocoPeriodicityGoalPair(forcePath));
end

problem.addGoal(periodicityGoal);

% Update the solver tolerances.
solver = MocoCasADiSolver.safeDownCast(study.updSolver());
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-4);
solver.resetProblem(problem);

% Solve!
solution = study.solve();
solution.write("exampleMocoTrack_state_tracking_solution.sto");

% Visualize the solution.
study.visualize(solution);

end

function muscleDrivenJointMomentTracking()

import org.opensim.modeling.*;

% Create and name an instance of the MocoTrack tool. 
track = MocoTrack();
track.setName("muscle_driven_joint_moment_tracking");

% Construct a ModelProcessor and set it on the tool.
modelProcessor = ModelProcessor("subject_walk_scaled.osim");
jointsToWeld = StdVectorString();
jointsToWeld.add("mtp_r");
jointsToWeld.add("mtp_l");
modelProcessor.append(ModOpReplaceJointsWithWelds(jointsToWeld));
modelProcessor.append(ModOpAddExternalLoads("grf_walk.xml"));
modelProcessor.append(ModOpAddResiduals(250.0, 50.0, 1.0));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(...
        "subject_walk_scaled_FunctionBasedPathSet.xml"));
track.setModel(modelProcessor);

% We will still track the coordinates trajectory, but with a lower weight.
track.setStatesReference(TableProcessor("coordinates.sto"));
track.set_states_global_tracking_weight(0.01);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);

% Initial time, final time, and mesh interval.
track.set_initial_time(0.48);
track.set_final_time(1.61);
track.set_mesh_interval(0.02);
    
% Get the underlying MocoStudy.
study = track.initialize();
problem = study.updProblem();

% Set the control effort weights.
effort = MocoControlGoal.safeDownCast(problem.updGoal("control_effort"));
effort.setWeight(0.1);
effort.setWeightForControlPattern('.*pelvis.*', 10);

% Constrain the states and controls to be periodic.
periodicityGoal = MocoPeriodicityGoal("periodicity");
model = modelProcessor.process();
model.initSystem();
for i = 0:model.getNumStateVariables()-1
    currentStateName = string(model.getStateVariableNames().getitem(i));
    if (~contains(currentStateName,"pelvis_tx/value"))
        periodicityGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
    end
end

forceSet = model.getForceSet();
for i = 0:forceSet.getSize()-1
    forcePath = forceSet.get(i).getAbsolutePathString();
    periodicityGoal.addControlPair(MocoPeriodicityGoalPair(forcePath));
end

problem.addGoal(periodicityGoal);

% Add a joint moment tracking goal to the problem.
jointMomentTracking = MocoGeneralizedForceTrackingGoal(...
        "joint_moment_tracking", 1e-2);

% Set the reference joint moments from an inverse dynamics solution and
% low-pass filter the data at 10 Hz. The reference data should use the 
% same column label format as the output of the Inverse Dynamics Tool.
jointMomentRef = TableProcessor("id_walk.sto");
jointMomentRef.append(TabOpLowPassFilter(10));
jointMomentTracking.setReference(jointMomentRef);

% Set the force paths that will be applied to the model to compute the
% generalized forces. Usually these are the external loads and actuators 
% (e.g., muscles) should be excluded, but any model force can be included 
% or excluded. Gravitational force is applied by default.
% Regular expression are supported when setting the force paths.
forcePaths = StdVectorString();
forcePaths.add(".*externalloads.*");
jointMomentTracking.setForcePaths(forcePaths);

% Allow unused columns in the reference data.
jointMomentTracking.setAllowUnusedReferences(true);

% Normalize the tracking error for each generalized for by the maximum 
% absolute value in the reference data for that generalized force.
jointMomentTracking.setNormalizeTrackingError(true);

% Ignore coordinates that are locked, prescribed, or coupled to other
% coordinates via CoordinateCouplerConstraints (true by default).
jointMomentTracking.setIgnoreConstrainedCoordinates(true);

% Do not track generalized forces associated with pelvis residuals.
jointMomentTracking.setWeightForGeneralizedForcePattern(".*pelvis.*", 0);

% Encourage better tracking of the ankle joint moments.
jointMomentTracking.setWeightForGeneralizedForce("ankle_angle_r_moment", 100);
jointMomentTracking.setWeightForGeneralizedForce("ankle_angle_l_moment", 100);

% Add the joint moment tracking goal to the problem.
problem.addGoal(jointMomentTracking);

% Update the solver problem and tolerances.
solver = MocoCasADiSolver.safeDownCast(study.updSolver());
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-4);
solver.resetProblem(problem);

% Set the guess, if available.
if (exist("exampleMocoTrack_state_tracking_solution.sto", "file"))
    study.setGuessFile("exampleMocoTrack_state_tracking_solution.sto");
end

% Solve!
solution = study.solve();
solution.write("exampleMocoTrack_joint_moment_tracking_solution.sto");

% Save the model to a file.
model.print("exampleMocoTrack_model.osim");

% Compute the joint moments and write them to a file.
forcePaths = StdVectorString();
forcePaths.add(".*externalloads.*");
jointMoments = study.calcGeneralizedForces(solution, forcePaths);
STOFileAdapter.write(jointMoments, "exampleMocoTrack_joint_moments.sto");

% Visualize the solution.
study.visualize(solution);

end


