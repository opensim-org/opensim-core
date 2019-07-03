function assistedSitToStand
% This file is for a workshop competition to design an assistive device for
% sit-to-stand that reduces control effort the most.
% evaluateDevice() returns the score for your device, which is the percent
% reduction in effort summed over 2 subjects, which have different mass
% properties.

% Competition rules:
% 1. Add any number of SpringGeneralizedForce components to the model.
% 2. The springs can be applied to the following coordinates:
%       hip_flexion_r
%       knee_angle_r
%       ankle_angle_r
% 3. Set Stiffness, RestLength, and Viscosity to any non-negative value.
% 4. The sum of all Stiffnesses must be less than 15.
% 5. Do not add any other types of components to the model.
% 6. Do not edit or remove any components already in the model.
%
% Information:
% 1. We started you off with two different device designs, stored in the
%    addSpringToKnee() and addSpringToAnkle() functions below.
% 2. To help with experimenting with different designs, create a separate
%    function for each design and change the argument to the evaluateDevice()
%    function below.
% 3. The unassisted solutions are cached as subject1_unassisted_solution.sto
%    and subject2_unassisted_solution.sto. If you want to re-run the unassisted
%    optimizations, delete these STO files or set cacheUnassisted to false.
% 4. To make Moco optimize the device parameters for you, do the following:
%       a. Create a MocoStudy using createStudy().
%       b. Add a MocoParameter to your problem representing
%       c. (optional) Set an initial guess for your parameter.
%          See createGuess() and setGuess() in MocoDirectCollocationSolver.
%       d. Solve the study returned from createStudy().
%       e. Get the parameter values out of the MocoSolution returned by solve().
%          See the documentation for MocoTrajectory.
%       f. Copy the parameter values into your device function, and evaluate
%          the optimized design.

import org.opensim.modeling.*;

global verbosity;
global visualize;
global cacheUnassisted;
createSubjectInfos();

% Use the verbosity variable to control console output (0 or 1).
verbosity = 1;
% Visualize the simulations after solving, and plot the solution trajectories.
visualize = 1;
% Avoid re-running the optimization for the unassisted cases.
cacheUnassisted = 1;

% Edit the argument to evaluateDevice() to any device function you create below.
score = evaluateDevice(@addSpringToKnee);

% Use this space to perform a parameter optimization, if you wish.

end

% Edit these device functions or create your own!
function name = addSpringToKnee(model)
name = 'knee_spring';
import org.opensim.modeling.*;
device = SpringGeneralizedForce('knee_angle_r');
device.setName('knee_spring');
device.setStiffness(15);
device.setRestLength(0);
device.setViscosity(0);
model.addForce(device);
end

function name = addSpringToAnkle(model)
name = 'ankle_spring';
import org.opensim.modeling.*;
device = SpringGeneralizedForce('ankle_angle_r');
device.setName('ankle_spring');
device.setStiffness(10);
device.setRestLength(0);
device.setViscosity(5);
model.addForce(device);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DO NOT EDIT BELOW THIS LINE                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function createSubjectInfos()
% Create a global struct that specifies the mass properties of the 2 subjects.
global subjectInfos;
subjectInfos{1} = createSubjectInfo(1);
subjectInfos{2} = createSubjectInfo(2);
subjectInfos{2}.torso = 1.5;
end

function [moco] = createStudy(subjectIndex, addDeviceFunction)
% This function builds a MocoProblem for predicting a sit-to-stand motion given
% adjustments to a model (given a subject index; 1 or 2) and, optionally, a
% function for adding a device to a model.
global verbosity;
global subjectInfos;

import org.opensim.modeling.*;

moco = MocoStudy();

% Configure the problem.
problem = moco.updProblem();

% Set the model.
subjectInfo = subjectInfos{subjectIndex};
ignoreActivationDynamics = true;
model = getMuscleDrivenModel(ignoreActivationDynamics, subjectInfo);

% Add the device to the model.
if nargin > 1
    name = addDeviceFunction(model);
    problem.setName([subjectInfo.name '_assisted_' name])
else
    problem.setName([subjectInfo.name '_unassisted'])
end
problem.setModel(model);

% Set variable bounds.
problem.setTimeBounds(0, 1);
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-2, 0.5], -2, 0);
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-2, 0], -2, 0);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', ...
    [-0.5, 0.7], -0.5, 0);
problem.setStateInfoPattern('/jointset/.*/speed', [], 0, 0);

% Set the cost.
problem.addCost(MocoControlCost('myeffort'));

% Configure the solver.
solver = moco.initCasADiSolver();
solver.set_dynamics_mode('implicit');
solver.set_num_mesh_points(25);
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-3);
solver.set_optim_finite_difference_scheme('forward');
solver.set_parameters_require_initsystem(false);
if ~verbosity
    solver.set_verbosity(0);
    solver.set_optim_ipopt_print_level(1);
end

end

function [solution] = solve(subjectInfo, addDeviceFunction)
% This function solves a MocoStudy created by createStudy() and may
% visualize the solution.
global visualize;

import org.opensim.modeling.*;

moco = createStudy(subjectInfo, addDeviceFunction);

solution = moco.solve();

if nargin > 1
    solution.write([char(problem.getName()) '_solution.sto']);
end

% outputPaths = StdVectorString();
% outputPaths.add('.*multiplier');
% outputTable = moco.analyze(solution, outputPaths);
% STOFileAdapter.write(outputTable, "assistedOutputs.sto");

if visualize
    moco.visualize(solution);
end

end


%% The remainder of the file contains utility functions.

function evaluateDevice(addDeviceFunction)
% This function runs unassisted and assisted optimizations on 2 subjects
% and prints the score for the device.

global visualize;
global cacheUnassisted;
global subjectInfos;

import org.opensim.modeling.*;

% Check the stiffness constraint.
if nargin > 1
    model = getMuscleDrivenModel(true, subjectInfos{1});
    name = addDeviceFunction(model);
    compList = model.getComponentsList();
    it = compList.begin();
    sumStiffness = 0;
    while ~it.equals(compList.end())
        if strcmp(it.getConcreteClassName(), 'SpringGeneralizedForce')
            object = model.getComponent(it.getAbsolutePathString());
            property = object.getPropertyByName('stiffness');
            stiffness = PropertyHelper.getValueDouble(property);
            sumStiffness = sumStiffness + stiffness;
        end
        it.next();
    end
    if sumStiffness > 15
        error('Sum of SpringGeneralizedForce stiffness must not exceed 15!');
    end
end

subjects = [1, 2];

percentChange = zeros(length(subjects), 1);

for subject = subjects
    info = subjectInfos{subject};
    str = sprintf('subject%i', subject);
    if cacheUnassisted && exist([str '_unassisted_solution.sto'], 'file')
        unassistedSolution = MocoTrajectory([str '_unassisted_solution.sto']);
        table = STOFileAdapter.read([str '_unassisted_solution.sto']);
        unassistedObjective = ...
            str2double(table.getTableMetaDataAsString('objective'));
    else
        unassistedSolution = solve(info);
        unassistedObjective = unassistedSolution.getObjective();
        unassistedSolution.write([str '_unassisted_solution.sto']);
    end

    assistedSolution = solve(info, addDeviceFunction);

    if visualize
        mocoPlotTrajectory(unassistedSolution, assistedSolution);
    end

    fprintf('Subject %i unassisted: %f\n', subject, unassistedObjective);
    fprintf('Subject %i assisted: %f\n', subject, assistedSolution.getObjective());

    percentChange(subject) = ...
            100.0 * (assistedSolution.getObjective() / unassistedObjective - 1);
end


for subject = subjects
    fprintf('Subject %i percent change: %f\n', subject, percentChange(subject));
end
fprintf('Score (lower is better): %f\n', sum(percentChange));

end


function subjectInfo = createSubjectInfo(number)
subjectInfo.name = sprintf('subject%i', number);
subjectInfo.torso = 1;
subjectInfo.femur_r = 1;
subjectInfo.tibia_r = 1;
end

