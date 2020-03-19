% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleMocoInverse.m                                         %
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

% This example shows how to use the MocoInverse tool to exactly prescribe a
% motion and estimate muscle behavior for walking. The first example does not
% rely on electromyography data, while the second example penalizes deviation
% from electromyography data for a subset of muscles.
%
% See the README.txt next to this file for more information.

function exampleMocoInverse()

% This problem solves in about 5 minutes.
solveMocoInverse();

% This problem penalizes the deviation from electromyography data for a
% subset of muscles, and solves in about 30 minutes.
solveMocoInverseWithEMG();

end

function solveMocoInverse()

import org.opensim.modeling.*;

% Construct the MocoInverse tool.
inverse = MocoInverse();

% Construct a ModelProcessor and set it on the tool. The default
% muscles in the model are replaced with optimization-friendly
% DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
% parameters.
modelProcessor = ModelProcessor('subject_walk_armless.osim');
modelProcessor.append(ModOpAddExternalLoads('grf_walk.xml'));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
modelProcessor.append(ModOpAddReserves(1.0));
inverse.setModel(modelProcessor);

% Construct a TableProcessor of the coordinate data and pass it to the
% inverse tool. TableProcessors can be used in the same way as
% ModelProcessors by appending TableOperators to modify the base table.
% A TableProcessor with no operators, as we have here, simply returns the
% base table.
inverse.setKinematics(TableProcessor('coordinates.sto'));

% Initial time, final time, and mesh interval.
inverse.set_initial_time(0.81);
inverse.set_final_time(1.79);
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
modelProcessor = ModelProcessor('subject_walk_armless.osim');
modelProcessor.append(ModOpAddExternalLoads('grf_walk.xml'));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
modelProcessor.append(ModOpAddReserves(1.0));
inverse.setModel(modelProcessor);
inverse.setKinematics(TableProcessor('coordinates.sto'));
inverse.set_initial_time(0.81);
inverse.set_final_time(1.79);
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
