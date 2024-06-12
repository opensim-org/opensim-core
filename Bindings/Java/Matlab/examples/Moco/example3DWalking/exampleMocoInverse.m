% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleMocoInverse.m                                         %
% -------------------------------------------------------------------------- %
% Copyright (c) 2023 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Christopher Dembia, Nicholas Bianco                             %
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

% This example shows how to use the MocoInverse tool to exactly prescribe a
% motion and estimate muscle behavior for walking. The first example does not
% rely on electromyography data, while the second example penalizes deviation
% from electromyography data for a subset of muscles. The third example
% extracts muscle synergies from the muscle excitaitons from the first example
% and uses them to solve the inverse problem using SynergyControllers.
%
% See the README.txt next to this file for more information.

function exampleMocoInverse()

% Solve the basic muscle redundancy problem with MocoInverse.
solveMocoInverse();

% This problem penalizes the deviation from electromyography data for a
% subset of muscles.
solveMocoInverseWithEMG();

% This problem extracts muscle synergies from the muscle excitations from
% the first example and uses them to solve the inverse problem using
% SynergyControllers.
numSynergies = 5;
solveMocoInverseWithSynergies(numSynergies);

end

function solveMocoInverse()

import org.opensim.modeling.*;

% Construct the MocoInverse tool.
inverse = MocoInverse();

% Construct a ModelProcessor and set it on the tool. The default
% muscles in the model are replaced with optimization-friendly
% DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
% parameters.
modelProcessor = ModelProcessor('subject_walk_scaled.osim');
modelProcessor.append(ModOpAddExternalLoads('grf_walk.xml'));
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
modelProcessor.append(ModOpAddReserves(1.0));
inverse.setModel(modelProcessor);

% Construct a TableProcessor of the coordinate data and pass it to the
% inverse tool. TableProcessors can be used in the same way as
% ModelProcessors by appending TableOperators to modify the base table.
% A TableProcessor with no operators, as we have here, simply returns the
% base table.
inverse.setKinematics(TableProcessor('coordinates.sto'));

% Initial time, final time, and mesh interval.
inverse.set_initial_time(0.48);
inverse.set_final_time(1.61);
inverse.set_mesh_interval(0.02);

% By default, Moco gives an error if the kinematics contains extra columns.
% Here, we tell Moco to allow (and ignore) those extra columns.
inverse.set_kinematics_allow_extra_columns(true);

% Solve the problem and write the solution to a Storage file.
solution = inverse.solve();
solution.getMocoSolution().write('example3DWalking_MocoInverse_solution.sto');

% Generate a report with plots for the solution trajectory.
model = modelProcessor.process();
report = osimMocoTrajectoryReport(model, ...
        'example3DWalking_MocoInverse_solution.sto', 'bilateral', true);
% The report is saved to the working directory.
reportFilepath = report.generate();
open(reportFilepath);

end

function solveMocoInverseWithEMG()

import org.opensim.modeling.*;

% This initial block of code is identical to the code above.
inverse = MocoInverse();
modelProcessor = ModelProcessor('subject_walk_scaled.osim');
modelProcessor.append(ModOpAddExternalLoads('grf_walk.xml'));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(...
        "subject_walk_scaled_FunctionBasedPathSet.xml"));
modelProcessor.append(ModOpAddReserves(1.0));
inverse.setModel(modelProcessor);
inverse.setKinematics(TableProcessor('coordinates.sto'));
inverse.set_initial_time(0.48);
inverse.set_final_time(1.61);
inverse.set_mesh_interval(0.02);
inverse.set_kinematics_allow_extra_columns(true);

study = inverse.initialize();
problem = study.updProblem();

% Add electromyography tracking.
emgTracking = MocoControlTrackingGoal('emg_tracking');
emgTracking.setWeight(50.0);
% Each column in electromyography.sto is normalized so the maximum value in
% each column is 1.0.
controlsRef = TimeSeriesTable('electromyography.sto');

% Scale down the tracked muscle activity based on peak levels from
% "Gait Analysis: Normal and Pathological Function" by
% Perry and Burnfield, 2010 (digitized by Carmichael Ong).
soleus = controlsRef.updDependentColumn('soleus');
gasmed = controlsRef.updDependentColumn('gastrocnemius');
tibant = controlsRef.updDependentColumn('tibialis_anterior');
for t = 0:controlsRef.getNumRows() - 1
    soleus.set(t, 0.77 * soleus.get(t));
    gasmed.set(t, 0.87 * gasmed.get(t));
    tibant.set(t, 0.37 * tibant.get(t));
end
emgTracking.setReference(TableProcessor(controlsRef));
% Associate actuators in the model with columns in electromyography.sto.
emgTracking.setReferenceLabel('/forceset/soleus_r', 'soleus')
emgTracking.setReferenceLabel('/forceset/gasmed_r', 'gastrocnemius')
emgTracking.setReferenceLabel('/forceset/gaslat_r', 'gastrocnemius')
emgTracking.setReferenceLabel('/forceset/tibant_r', 'tibialis_anterior')
problem.addGoal(emgTracking)

% Solve the problem and write the solution to a Storage file.
solution = study.solve();
solution.write('example3DWalking_MocoInverseWithEMG_solution.sto');

% Write the reference data in a way that's easy to compare to the solution.
controlsRef.removeColumn('medial_hamstrings');
controlsRef.removeColumn('biceps_femoris');
controlsRef.removeColumn('vastus_lateralis');
controlsRef.removeColumn('vastus_medius');
controlsRef.removeColumn('rectus_femoris');
controlsRef.removeColumn('gluteus_maximus');
controlsRef.removeColumn('gluteus_medius');
columnLabels = StdVectorString();
columnLabels.add('/forceset/soleus_r');
columnLabels.add('/forceset/gasmed_r');
columnLabels.add('/forceset/tibant_r');
controlsRef.setColumnLabels(columnLabels);
controlsRef.appendColumn('/forceset/gaslat_r', gasmed);
STOFileAdapter.write(controlsRef, 'controls_reference.sto');

% Generate a report comparing MocoInverse solutions without and with EMG
% tracking.
model = modelProcessor.process();
report = osimMocoTrajectoryReport(model, ...
        'example3DWalking_MocoInverse_solution.sto', ...
        'outputFilepath', 'example3DWalking_MocoInverseWithEMG_report.pdf', ...
        'bilateral', true, ...
        'refFiles', {'example3DWalking_MocoInverseWithEMG_solution.sto', ...
                     'controls_reference.sto'});
% The report is saved to the working directory.
reportFilepath = report.generate();
open(reportFilepath);

end

function solveMocoInverseWithSynergies(numSynergies)

import org.opensim.modeling.*;

% Construct the base model using a ModelProcessor as in the previous
% examples, with the exception that we ignore activation dynamics to
% simplify the problem given the muscle synergy controllers.
modelProcessor = ModelProcessor('subject_walk_scaled.osim');
modelProcessor.append(ModOpAddExternalLoads('grf_walk.xml'));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpIgnoreActivationDynamics());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(...
        "subject_walk_scaled_FunctionBasedPathSet.xml"));
modelProcessor.append(ModOpAddReserves(1.0));
model = modelProcessor.process();

% Load the solution from solveMocoInverse() to extract the muscle
% control variable names and excitations for the left and right legs.
prevSolution = MocoTrajectory('example3DWalking_MocoInverse_solution.sto');
leftControlNames = {};
rightControlNames = {};
model.initSystem();
forceSet = model.getForceSet();
for i = 1:forceSet.getSize()
    force = forceSet.get(i-1);
    name = force.getName();
    if endsWith(force.getConcreteClassName(), 'Muscle')
        if endsWith(name, '_r') 
            index = length(rightControlNames)+1;
            rightControlNames{index} = force.getAbsolutePathString();
        elseif endsWith(name, '_l')
            index = length(leftControlNames)+1;
            leftControlNames{index} = force.getAbsolutePathString();
        end
    end

end

controls = prevSolution.exportToControlsTable();
leftControls = TimeSeriesTable(controls.getIndependentColumn());
rightControls = TimeSeriesTable(controls.getIndependentColumn());
for i = 1:length(leftControlNames)
    leftControls.appendColumn(leftControlNames{i}, ...
            controls.getDependentColumn(leftControlNames{i}));
end
for i = 1:length(rightControlNames)
    rightControls.appendColumn(rightControlNames{i}, ...
            controls.getDependentColumn(rightControlNames{i}));
end

% SynergyController
% -----------------
% SynergyController defines the controls for actuators connected to the 
% controller using a linear combination of time-varying synergy control 
% signals (i.e., "synergy excitations") and a set of vectors containing 
% weights for each actuator representing the contribution of each synergy
% excitation to the total control signal for that actuator 
% (i.e., "synergy vectors").
%
% If 'N' is the number of time points in the trajectory, 'M' is the number
% of actuators connected to the controller, and 'K' is the number of   
% synergies in the controller, then:
% - The synergy excitations are a matrix 'W' of size N x K.
% - The synergy vectors are a matrix 'H' of size K x M.
% - The controls for the actuators are computed A = W * H.
%  
% SynergyController is a concrete implementation of InputController, which
% means that it uses Input control signals as the synergy excitations.
% Moco automatically detects InputController%s in a model provided to a
% MocoProblem and adds continuous variables to the optimization problem
% for each Input control signal. The variable names are based on the path
% to the controller appended with the Input control labels (e.g.,
% "/path/to/my_synergy_controller/synergy_excitation_0").

% Use non-negative matrix factorization (NNMF) to extract a set of muscle
% synergies for each leg.
Al = leftControls.getMatrix().getAsMat();
[Wl, Hl] = nnmf(Al, numSynergies);

Ar = rightControls.getMatrix().getAsMat();
[Wr, Hr] = nnmf(Ar, numSynergies);

% Scale W and H assuming that the elements of H are all 0.5.
scaleVec = 0.5*ones(1, length(leftControlNames));
for i = 1:numSynergies
    scale_l = norm(scaleVec) / norm(Hl(i, :));
    Hl(i, :) = Hl(i, :) * scale_l;
    Wl(:, i) = Wl(:, i) / scale_l;

    scale_r = norm(scaleVec) / norm(Hr(i, :));
    Hr(i, :) = Hr(i, :) * scale_r;
    Wr(:, i) = Wr(:, i) / scale_r;
end

% Add a SynergyController for the left leg to the model.
leftController = SynergyController();
leftController.setName("synergy_controller_left_leg");
% The number of actuators connected to the controller defines the number of
% weights in each synergy vector expected by the controller.
for i = 1:length(leftControlNames)
    leftController.addActuator(...
            Muscle.safeDownCast(model.getComponent(leftControlNames{i})));
end
% Adding a synergy vector increases the number of synergies in the
% controller by one. This means that the number of Input control 
% signals expected by the controller is also increased by one.
for i = 1:numSynergies  
    synergyVector = Vector(length(leftControlNames), 0.0);
    for j = 1:length(leftControlNames)
        synergyVector.set(j-1, Hl(i, j));
    end
    leftController.addSynergyVector(synergyVector);
end
model.addController(leftController);

% Add a SynergyController for the right leg to the model.
rightController = SynergyController();
rightController.setName("synergy_controller_right_leg");
for i = 1:length(rightControlNames)
    rightController.addActuator(...
            Muscle.safeDownCast(model.getComponent(rightControlNames{i})));
end
for i = 1:numSynergies  
    synergyVector = Vector(length(rightControlNames), 0.0);
    for j = 1:length(rightControlNames)
        synergyVector.set(j-1, Hr(i, j));
    end
    rightController.addSynergyVector(synergyVector);
end
model.addController(rightController);
model.finalizeConnections();
model.initSystem();

% Construct the MocoInverse tool.
inverse = MocoInverse();
inverse.setName("example3DWalking_MocoInverseWithSynergies");
inverse.setModel(ModelProcessor(model));
inverse.setKinematics(TableProcessor('coordinates.sto'));
inverse.set_initial_time(0.48);
inverse.set_final_time(1.61);
inverse.set_mesh_interval(0.02);
inverse.set_kinematics_allow_extra_columns(true);

% Initialize the MocoInverse study and set the control bounds for the
% muscle synergies excitations. 'setInputControlInfo()' is equivalent to
% 'setControlInfo()', but reserved for Input control variables. Note that 
% while we make a distinction between "control" variables and 
% "Input control" variables in the API, the optimal control problem
% constructed by Moco treats them both as algebraic variables.
study = inverse.initialize();
problem = study.updProblem();

% We will also increase the weight on the synergy excitations in the 
% control effort cost term. MocoControlGoal, and other MocoGoals, that 
% depend on control variables have options configuring cost terms with
% Input control values.
effort = MocoControlGoal.safeDownCast(problem.updGoal("excitation_effort"));
for i = 0:numSynergies-1
    nameLeft = ['/controllerset/synergy_controller_left_leg' ...
                '/synergy_excitation_' num2str(i)];
    problem.setInputControlInfo(nameLeft, [0, 1.0])
    effort.setWeightForControl(nameLeft, 10)

    nameRight = ['/controllerset/synergy_controller_right_leg' ...
                    '/synergy_excitation_' num2str(i)];
    problem.setInputControlInfo(nameRight, [0, 1.0])
    effort.setWeightForControl(nameRight, 10)
end

% Solve!
solution = study.solve();

% This function computes the model control values from the
% SynergyControllers in the model and inserts them into the solution.
solution.generateControlsFromModelControllers(model);

% Add the multibody states into the solutions so we can visualize the
% trajectory or utilize the plotting utilities.
coordinateValues = prevSolution.exportToValuesTable();
coordinateSpeeds = prevSolution.exportToSpeedsTable();
solution.insertStatesTrajectory(coordinateValues);
solution.insertStatesTrajectory(coordinateSpeeds);

% Write the solution to a Storage file.
solutionFile =['example3DWalking_MocoInverseWith' num2str(numSynergies) ...
                'Synergies_solution.sto'];
solution.write(solutionFile)

% Generate a report comparing MocoInverse solutions with and without
% muscle synergies.
model = modelProcessor.process();
report = osimMocoTrajectoryReport(model, solutionFile, ...
        'outputFilepath', ...
        'example3DWalking_MocoInverseWithSynergies_report.pdf', ...
        'bilateral', true, ...
        'refFiles', {'example3DWalking_MocoInverse_solution.sto'});
% The report is saved to the working directory.
reportFilepath = report.generate();
open(reportFilepath);

end