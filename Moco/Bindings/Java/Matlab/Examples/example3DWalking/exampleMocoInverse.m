% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleMocoInverse.m                                         %
% -------------------------------------------------------------------------- %
% Copyright (c) 2019 Stanford University and the Authors                     %
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
% motion and estimate muscle behavior for walking.
% This problem solves in about 5 minutes.
%
% See the README.txt next to this file for more information.

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
