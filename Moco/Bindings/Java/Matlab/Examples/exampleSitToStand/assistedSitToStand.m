function assistedSitToStand
import org.opensim.modeling.*;

% TODO: Use analysis to see if glut passive forces are too large.

subject1Info = createSubjectInfo();

subject2Info = createSubjectInfo();
subject2Info.psoas_r = 1.2;
subject2Info.vas_int_r = 0.8;

if ~exist('subject1UnassistedSolution.sto', 'file')
    subject1UnassistedSolution = solve(subject1Info);
    subject1UnassistedObjective = subject1UnassistedSolution.getObjective();
    subject1UnassistedSolution.write('subject1UnassistedSolution.sto');
else
    subject1UnassistedSolution = MocoTrajectory('subject1UnassistedSolution.sto');
    table = STOFileAdapter.read('subject1UnassistedSolution.sto');
    subject1UnassistedObjective = ...
        str2double(table.getTableMetaDataAsString('objective'));
end
if ~exist('subject2UnassistedSolution.sto', 'file')
    subject2UnassistedSolution = solve(subject2Info);
    subject2UnassistedObjective = subject2UnassistedSolution.getObjective();
    subject2UnassistedSolution.write('subject2UnassistedSolution.sto');
else
    subject2UnassistedSolution = MocoTrajectory('subject2UnassistedSolution.sto');
    table = STOFileAdapter.read('subject2UnassistedSolution.sto');
    subject2UnassistedObjective = ...
        str2double(table.getTableMetaDataAsString('objective'));
end

subject1AssistedSolution = solve(subject1Info, @addSpringToKnee);
subject2AssistedSolution = solve(subject2Info, @addSpringToKnee);

mocoPlotTrajectory(subject1UnassistedSolution, subject1AssistedSolution);
mocoPlotTrajectory(subject2UnassistedSolution, subject2AssistedSolution);

fprintf('Subject 1 unassisted: %f\n', subject1UnassistedObjective);
fprintf('Subject 1 assisted: %f\n', subject1AssistedSolution.getObjective());
fprintf('Subject 1 unassisted: %f\n', subject2UnassistedObjective);
fprintf('Subject 1 assisted: %f\n', subject2AssistedSolution.getObjective());

end

% TODO: Give users examples of other devices they can add.
% hip_flexion_r
% knee_angle_r
% ankle_angle_r
function addSpringToKnee(model)
import org.opensim.modeling.*;
device = SpringGeneralizedForce('knee_angle_r');
device.setStiffness(50);
device.setRestLength(0);
device.setViscosity(0);
model.addForce(device);
end

function subjectInfo = createSubjectInfo()
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
import org.opensim.modeling.*;
moco = MocoStudy();

problem = moco.updProblem();
model = getMuscleDrivenModel(subjectInfo);
if nargin > 1
    addDeviceFunction(model);
end
problem.setModel(model);

problem.setTimeBounds(0, 1);
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-2, 0.5], -2, 0);
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-2, 0], -2, 0);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', ...
    [-0.5, 0.7], -0.5, 0);
problem.setStateInfoPattern('/jointset/.*/speed', [], 0, 0);
problem.setStateInfoPattern('.*/activation', [], 0);

problem.addCost(MocoControlCost('myeffort'));

solver = moco.initCasADiSolver();
solver.set_dynamics_mode('implicit');
solver.set_num_mesh_points(25);
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-3);
solver.set_optim_finite_difference_scheme('forward');

solution = moco.solve();
end

function [model] = getMuscleDrivenModel(subjectInfo)

% TODO modify model according to subject info.

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
    musc = model.updMuscles().get(fields{ifield});
    origFmax = musc.getMaxIsometricForce();
    factor = subjectInfo.(fields{ifield});
    musc.setMaxIsometricForce(factor * origFmax);
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
