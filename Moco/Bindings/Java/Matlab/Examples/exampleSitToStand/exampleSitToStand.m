function exampleSitToStand

import org.opensim.modeling.*;

model = getTorqueDrivenModel();
moco = configureMocoTool(model);
problem = moco.updProblem();

problem.addCost(MocoControlCost());

% solver = MocoCasADiSolver().safeDownCast(moco.updSolver());
% guess = solver.createGuess('bounds');

moco.solve();

end

function [moco] = configureMocoTool(model)

import org.opensim.modeling.*;

moco = MocoTool();

problem = moco.updProblem();

problem.setModelCopy(createModel('torques'));

problem.setTimeBounds(0, 1);
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', ...
    MocoBounds(-2, 0.5), MocoInitialBounds(-2), MocoFinalBounds(0));
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', ...
    [-2, 0], -2, 0);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', ...
    [-0.5, 0.7], -0.5, 0);

solver = moco.initCasADiSolver();
solver.set_num_mesh_points(25);
solver.set_dynamics_mode('implicit');
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-4);
solver.set_optim_solver('ipopt');
solver.set_transcription_scheme('hermite-simpson');
solver.set_enforce_constraint_derivatives(true);
solver.set_optim_hessian_approximation('limited-memory');

end

function [model] = getTorqueDrivenModel()

import org.opensim.modeling.*;

% Load the base model.
model = Model('sitToStand_3dof9musc.osim');

% Remove the muscles in the model.
model.updForceSet().clearAndDestroy();
model.initSystem();

% Add CoordinateActuators to the model degrees-of-freedom.
addCoordinateActuator(model, 'hip_flexion_r', 250);
addCoordinateActuator(model, 'knee_angle_r', 500);
addCoordinateActuator(model, 'ankle_angle_r', 250);

end

function addCoordinateActuator(model, coordName, optForce)

import org.opensim.modeling.*;

coordSet = model.updCoordinateSet();

actu = CoordinateActuator();
actu.setName(['tau_' coordName]);
actu.setCoordinate(coordSet.get(coordName));
actu.setOptimalForce(optForce);
actu.setMinControl(-1);
actu.setMaxControl(1);

model.addComponent(actu);

end

% function [model] = getMuscleDrivenModel()
% 
% import org.opensim.modeling.*;
% 
% % Load the base model.
% model = Model('sitToStand_3dof9musc.osim');
% 
% model.finalizeConnections();
% DeGrooteFregly2016Muscle().replaceMuscles(model);
% for m = 0:model.getMuscles()-1
%     musc = model.updMuscles().get(m);
%     musc.set_ignore_activation_dynamics(true);
%     musc.set_ignore_tendon_compliance(true);
% end
% 
% end