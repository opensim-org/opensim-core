function assistedSitToStand

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
%    addSpringToKnee() and addSpringToAnkle functions() below.
% 2. To help with experimenting with different designs, create a separate
%    function for each design and change the argument to the evaluateDevice()
%    function below.
% 3. The unassisted solutions are cached as subject1_unassisted_solution.sto
%    and subject2_unassisted_solution.sto. If you want to re-run the unassisted
%    optimizations, delete these STO files.

global verbosity;

% Use the verbosity variable to control console output (0 or 1).
verbosity = 1;

% TODO add options for visualizing
% TODO add options for creating additional subjects.

evaluateDevice(@addSpringToKnee);

end

function name = addSpringToKnee(model)
name = 'knee_spring';
import org.opensim.modeling.*;
device = SpringGeneralizedForce('knee_angle_r');
device.setStiffness(15);
device.setRestLength(0);
device.setViscosity(0);
model.addForce(device);
end

function name = addSpringToAnkle(model)
name = 'ankle_spring';
import org.opensim.modeling.*;
device = SpringGeneralizedForce('ankle_angle_r');
device.setStiffness(10);
device.setRestLength(0);
device.setViscosity(5);
model.addForce(device);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DO NOT EDIT BELOW THIS LINE                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function evaluateDevice(addDeviceFunction)
import org.opensim.modeling.*;

subjectInfos{1} = createSubjectInfo(1);
subjectInfos{1}.tib_ant_r = 0.5;

subjectInfos{2} = createSubjectInfo(2);
subjectInfos{2}.vas_int_r = 0.8;

subjects = [1, 2];

doCache = true;

for subject = subjects
    info = subjectInfos{subject};
    str = sprintf('subject%i', subject);
    if doCache && exist([str '_unassisted_solution.sto'], 'file')
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

    mocoPlotTrajectory(unassistedSolution, assistedSolution);

    fprintf('Subject %i unassisted: %f\n', subject, unassistedObjective);
    fprintf('Subject %i assisted: %f\n', subject, assistedSolution.getObjective());

    percentChange{subject} = ...
            100.0 * (assistedSolution.getObjective() / unassistedObjective - 1);
end


for subject = subjects
    fprintf('Subject %i percent change: %f\n', subject, percentChange{subject});
end

end


function subjectInfo = createSubjectInfo(number)
subjectInfo.name = sprintf('subject%i', number);
subjectInfo.bifemsh_r = 1;
subjectInfo.med_gas_r = 1;
subjectInfo.glut_max2_r = 1;
subjectInfo.psoas_r = 1;
subjectInfo.rect_fem_r = 1;
subjectInfo.semimem_r = 1;
subjectInfo.soleus_r = 1;
subjectInfo.tib_ant_r = 1;
subjectInfo.vas_int_r = 1;
end

function [solution] = solve(subjectInfo, addDeviceFunction)
global verbosity;
import org.opensim.modeling.*;
moco = MocoStudy();

problem = moco.updProblem();
model = getMuscleDrivenModel(subjectInfo);
if nargin > 1
    name = addDeviceFunction(model);
    problem.setName([subjectInfo.name '_assisted_' name])
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
else
    problem.setName([subjectInfo.name '_unassisted'])
end
problem.setModel(model);

problem.setTimeBounds(0, 1);
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-2, 0.5], -2, 0);
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-2, 0], -2, 0);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', ...
    [-0.5, 0.7], -0.5, 0);
problem.setStateInfoPattern('/jointset/.*/speed', [], 0, 0);


problem.addCost(MocoControlCost('myeffort'));

solver = moco.initCasADiSolver();
solver.set_dynamics_mode('implicit');
solver.set_num_mesh_points(25);
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-3);
solver.set_optim_finite_difference_scheme('forward');
if ~verbosity
    solver.set_verbosity(0);
    solver.set_optim_ipopt_print_level(1);
end

solution = moco.solve();

if nargin > 1
    solution.write([char(problem.getName()) '_solution.sto']);
end

% outputPaths = StdVectorString();
% outputPaths.add('.*multiplier');
% outputTable = moco.analyze(solution, outputPaths);
% STOFileAdapter.write(outputTable, "assistedOutputs.sto");

% moco.visualize(solution);
end

function [model] = getMuscleDrivenModel(subjectInfo)

import org.opensim.modeling.*;

% Load the base model.
model = Model('sitToStand_3dof9musc.osim');
model.finalizeConnections();

% Replace the muscles in the model with muscles from DeGroote, Fregly,
% et al. 2016, "Evaluation of Direct Collocation Optimal Control Problem
% Formulations for Solving the Muscle Redundancy Problem". These muscles
% have the same properties as the original muscles but their characteristic
% curves are optimized for direct collocation (i.e. no discontinuities,
% twice differentiable, etc).
DeGrooteFregly2016Muscle().replaceMuscles(model);

fields = fieldnames(subjectInfo);
for ifield = 1:numel(fields)
    if ~strcmp(fields{ifield}, 'name')
        musc = model.updMuscles().get(fields{ifield});
        origFmax = musc.getMaxIsometricForce();
        factor = subjectInfo.(fields{ifield});
        musc.setMaxIsometricForce(factor * origFmax);
    end
end

% Turn off activation dynamics and muscle-tendon dynamics to keep the
% problem simple.
for m = 0:model.getMuscles().getSize()-1
    musc = model.updMuscles().get(m);
    musc.setMinControl(0);
    musc.set_ignore_activation_dynamics(true);
    musc.set_ignore_tendon_compliance(true);
    musc.set_max_isometric_force(2 * musc.get_max_isometric_force());
    dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
    dgf.set_active_force_width_scale(1.5);
    if strcmp(char(musc.getName()), 'soleus_r')
        % Soleus has a very long tendon, so modeling its tendon as rigid
        % causes the fiber to be unrealistically long and generate
        % excessive passive fiber force.
        dgf.set_ignore_passive_fiber_force(true);
    end
end

end
