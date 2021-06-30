% -------------------------------------------------------------------------- %
% OpenSim Moco: example2DWalkingMetabolics.m                                 %
% -------------------------------------------------------------------------- %
% Copyright (c) 2021 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Brian Umberger                                                  %
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

% This is a Matlab implementation of an example optimal control
% problem (2-D walking) orginally created in C++ by Antoine Falisse
% (see: example2DWalkingMetabolics.cpp).
%
% This example features a tracking simulation of walking that includes
% minimization of the metabolic cost of transport computed using a smooth
% approximation of the metabolic energy model of Bhargava et al (2004).
%
% The code is inspired from Falisse A, Serrancoli G, Dembia C, Gillis J,
% De Groote F: Algorithmic differentiation improves the computational
% efficiency of OpenSim-based trajectory optimization of human movement.
% PLOS One, 2019.
%
% Model
% -----
% The model described in the file '2D_gait.osim' included in this file is a
% modified version of the 'gait10dof18musc.osim' available within OpenSim. We
% replaced the moving knee flexion axis by a fixed flexion axis, replaced the
% Millard2012EquilibriumMuscles by DeGrooteFregly2016Muscles, and added
% SmoothSphereHalfSpaceForces (two contacts per foot) to model the
% contact interactions between the feet and the ground.
%
% Data
% ----
% The coordinate data included in the 'referenceCoordinates.sto' comes from
% predictive simulations generated in Falisse et al. 2019.

clear;

% Load the Moco libraries
import org.opensim.modeling.*;

% Set a coordinate tracking problem where the goal is to minimize the
% difference between provided and simulated coordinate values and speeds
% as well as to minimize an effort cost (squared controls) and a metabolic
% cost (metabolic energy normalized by distance traveled and body mass;
% the metabolics model is based on a smooth approximation of the
% phenomenological model described by Bhargava et al. (2004)). The provided
% data represents half a gait cycle. Endpoint constraints enforce periodicity
% of the coordinate values (except for pelvis tx) and speeds, coordinate
% actuator controls, and muscle activations.

track = MocoTrack();
track.setName('gaitTrackingMetCost');

% Define the optimal control problem
% ==================================
model = Model('2D_gait.osim');

% Add metabolic cost model
metabolics = Bhargava2004SmoothedMuscleMetabolics();
metabolics.setName('metabolic_cost');
metabolics.set_use_smoothing(true);

% This next part can easily be put in a loop for models with more muscles
metabolics.addMuscle('hamstrings_r', Muscle.safeDownCast(model.getComponent('hamstrings_r')));
metabolics.addMuscle('hamstrings_l', Muscle.safeDownCast(model.getComponent('hamstrings_l')));
metabolics.addMuscle('bifemsh_r', Muscle.safeDownCast(model.getComponent('bifemsh_r')));
metabolics.addMuscle('bifemsh_l', Muscle.safeDownCast(model.getComponent('bifemsh_l')));
metabolics.addMuscle('glut_max_r', Muscle.safeDownCast(model.getComponent('glut_max_r')));
metabolics.addMuscle('glut_max_l', Muscle.safeDownCast(model.getComponent('glut_max_l')));
metabolics.addMuscle('iliopsoas_r', Muscle.safeDownCast(model.getComponent('iliopsoas_r')));
metabolics.addMuscle('iliopsoas_l', Muscle.safeDownCast(model.getComponent('iliopsoas_l')));
metabolics.addMuscle('rect_fem_r', Muscle.safeDownCast(model.getComponent('rect_fem_r')));
metabolics.addMuscle('rect_fem_l', Muscle.safeDownCast(model.getComponent('rect_fem_l')));
metabolics.addMuscle('vasti_r', Muscle.safeDownCast(model.getComponent('vasti_r')));
metabolics.addMuscle('vasti_l', Muscle.safeDownCast(model.getComponent('vasti_l')));
metabolics.addMuscle('gastroc_r', Muscle.safeDownCast(model.getComponent('gastroc_r')));
metabolics.addMuscle('gastroc_l', Muscle.safeDownCast(model.getComponent('gastroc_l')));
metabolics.addMuscle('soleus_r', Muscle.safeDownCast(model.getComponent('soleus_r')));
metabolics.addMuscle('soleus_l', Muscle.safeDownCast(model.getComponent('soleus_l')));
metabolics.addMuscle('tib_ant_r', Muscle.safeDownCast(model.getComponent('tib_ant_r')));
metabolics.addMuscle('tib_ant_l', Muscle.safeDownCast(model.getComponent('tib_ant_l')));

model.addComponent(metabolics);
model.finalizeConnections();

% Pass the model to MocoTrack
modelProcessor = ModelProcessor(model);
track.setModel(modelProcessor);

% Reference data for tracking problem
tableProcessor = TableProcessor('referenceCoordinates.sto');
tableProcessor.append(TabOpLowPassFilter(6));
track.setStatesReference(tableProcessor);

% Provide the remaining necesssary MocoTrack settings.
track.set_states_global_tracking_weight(30);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
track.set_initial_time(0.0);
track.set_final_time(0.47008941);

% Call initialize() to get the internal MocoStudy. This will allow us to
% make further modifications to the MocoProblem.
study = track.initialize();
problem = study.updProblem();

% Goals
% =====

% Symmetry
% --------
% This goal allows us to simulate only one step with left-right symmetry
% that we can then double to create a full gait cycle.
symmetryGoal = MocoPeriodicityGoal('symmetryGoal');
problem.addGoal(symmetryGoal);
model = modelProcessor.process();
model.initSystem();

% Symmetric coordinate values (except for pelvis_tx) and speeds. Here, we 
% constrain final coordinate values of one leg to match the initial value of the 
% other leg. Or, in the case of the pelvis_tx value, we constrain the final 
% value to be the same as the initial value.
for i = 1:model.getNumStateVariables()
    currentStateName = string(model.getStateVariableNames().getitem(i-1));
    if startsWith(currentStateName , '/jointset')
        if contains(currentStateName,'_r')
            pair = MocoPeriodicityGoalPair(currentStateName, ...
                           regexprep(currentStateName,'_r','_l'));
            symmetryGoal.addStatePair(pair);
        end
        if contains(currentStateName,'_l')
            pair = MocoPeriodicityGoalPair(currentStateName, ...
                           regexprep(currentStateName,'_l','_r'));
            symmetryGoal.addStatePair(pair);
        end
        if (~contains(currentStateName,'_r') && ...
            ~contains(currentStateName,'_l') && ...
            ~contains(currentStateName,'pelvis_tx/value') && ...
            ~contains(currentStateName,'/activation'))
            symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
        end
    end
end

% Symmetric muscle activations. Here, we constrain final muscle activation 
% values of one leg to match the initial activation values of the other leg.
for i = 1:model.getNumStateVariables()
    currentStateName = string(model.getStateVariableNames().getitem(i-1));
    if endsWith(currentStateName,'/activation')
        if contains(currentStateName,'_r')
            pair = MocoPeriodicityGoalPair(currentStateName, ...
                         regexprep(currentStateName,'_r','_l'));
            symmetryGoal.addStatePair(pair);
        end
        if contains(currentStateName,'_l')
            pair = MocoPeriodicityGoalPair(currentStateName, ...
                         regexprep(currentStateName,'_l','_r'));
            symmetryGoal.addStatePair(pair);
        end
    end
end

% The lumbar coordinate actuator control is symmetric.
symmetryGoal.addControlPair(MocoPeriodicityGoalPair('/lumbarAct'));

% Get a reference to the MocoControlGoal that is added to every MocoTrack
% problem by default and change the weight
effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
effort.setWeight(0.1);

% Metabolic cost; total metabolic rate includes activation heat rate,
% maintenance heat rate, shortening heat rate, mechanical work rate, and
% basal metabolic rate.
metGoal = MocoOutputGoal('met',0.1);
problem.addGoal(metGoal);
metGoal.setOutputPath('/metabolic_cost|total_metabolic_rate');
metGoal.setDivideByDisplacement(true);
metGoal.setDivideByMass(true);

% Bounds
% ======
problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-20*pi/180, -10*pi/180]);
problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 1]);
problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value', [0.75, 1.25]);
problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value', [-10*pi/180, 60*pi/180]);
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-10*pi/180, 60*pi/180]);
problem.setStateInfo('/jointset/knee_l/knee_angle_l/value', [-50*pi/180, 0]);
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-50*pi/180, 0]);
problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value', [-15*pi/180, 25*pi/180]);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [-15*pi/180, 25*pi/180]);
problem.setStateInfo('/jointset/lumbar/lumbar/value', [0, 20*pi/180]);

% Configure the solver.
% =====================
solver = MocoCasADiSolver.safeDownCast(study.updSolver());
solver.resetProblem(problem);
solver.set_num_mesh_intervals(50);
solver.set_verbosity(2);
solver.set_optim_solver("ipopt");
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-4);
solver.set_optim_max_iterations(10000);

% Solve the problem
% =================
solution = study.solve();

% Create a full stride from the periodic single step solution.
% For details, view the Doxygen documentation for createPeriodicTrajectory().
fullStride = opensimMoco.createPeriodicTrajectory(solution);
fullStride.write('gaitTrackingMetCost_solution_fullcycle.sto');

% To report the COT we multiply the metabolic cost objective term by 10 because
% it had been scaled by 0.1
disp('   ')
disp( ['The metabolic cost of transport is: '  ...
      num2str(10*solution.getObjectiveTerm('met')) ...
      ' J/kg/m'])
disp('   ')

% Visualize the result.
study.visualize(fullStride);

% Extract ground reaction forces
% ==============================
contact_r = StdVectorString();
contact_l = StdVectorString();
contact_r.add('contactHeel_r');
contact_r.add('contactFront_r');
contact_l.add('contactHeel_l');
contact_l.add('contactFront_l');

% Create a conventional ground reaction forces file by summing the contact
% forces of contact spheres on each foot.
% For details, view the Doxygen documentation for
% createExternalLoadsTableForGait().
externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(model, ...
                             fullStride, contact_r, contact_l);
STOFileAdapter.write(externalForcesTableFlat, ...
                             'gaitTrackingMetCost_solutionGRF_fullStride.sto');