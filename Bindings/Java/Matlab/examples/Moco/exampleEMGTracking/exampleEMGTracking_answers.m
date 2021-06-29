function exampleEMGTracking_answers
clear; close all; clc;

%% Part 0: Load the OpenSim and Moco libraries.
import org.opensim.modeling.*;

%% Part 1: Muscle redundancy problem: effort minimization.
% Solve the muscle redundancy problem while minimizing muscle excitations
% squared using the MocoInverse tool. 

% Part 1a: Load a 19 degree-of-freedom model with 18 lower-limb, 
% sagittal-plane muscles and a torque-actuated torso. This includes a set of 
% ground reaction forces applied to the model via ExternalLoads, which is 
% necessary for the muscle redundancy problem. See the function definition 
% at the bottom of this file to see how the model is loaded and constructed.
model = getWalkingModel(); 

% Part 1b: Create the MocoInverse tool and set the Model.
inverse = MocoInverse();
inverse.setModel(model);

% Part 1c: Create a TableProcessor using the coordinates file from inverse
% kinematics.
coordinates = TableProcessor('coordinates.mot');
coordinates.append(TabOpLowPassFilter(6));
coordinates.append(TabOpUseAbsoluteStateNames());

% Part 1d: Set the kinematics reference for MocoInverse using the 
% TableProcessor we just created.
inverse.setKinematics(coordinates);
inverse.set_kinematics_allow_extra_columns(true);

% Part 1e: Provide the solver settings: initial and final time, the mesh 
% interval, and the constraint and convergence tolerances.
inverse.set_initial_time(0.83);
inverse.set_final_time(2.0);
inverse.set_mesh_interval(0.04);
inverse.set_constraint_tolerance(1e-3);
inverse.set_convergence_tolerance(1e-3);

if ~exist('effortSolution.sto', 'file')
    % Part 1f: Solve the problem!
    inverseSolution = inverse.solve();
    solution = inverseSolution.getMocoSolution();
    solution.write('effortSolution.sto');
end

%% Part 2: Plot the muscle redundancy problem solution.
% Load the experimental electromyography data and compare 
% the effort minimization solution against this data. We will also use
% it later for the EMG-tracking problem. Each column in emg.sto is 
% normalized so the maximum value for each signal is 1.0.
emgReference = TimeSeriesTable('emg.sto');
compareSolutionToEMG(emgReference, 'effortSolution.sto');

%% Part 3: Muscle redundancy problem: EMG-tracking.
% Modify the existing problem we created with the MocoInverse tool to solve
% a new problem where we will track electromyography (EMG) data. 

% Part 3a: Call initialize() to get access to the MocoStudy contained within
% the MocoInverse instance. This will allow us to make additional
% modifications to the problem not provided by MocoInverse.
study = inverse.initialize();
problem = study.updProblem();

% Part 3b: Create a MocoControlTrackingGoal, set its weight, and provide
% the EMG data as the tracking reference. We also need to specify the 
% reference labels for the four muscles whose EMG we will track.
tracking = MocoControlTrackingGoal('emg_tracking');
tracking.setWeight(5);
tracking.setReference(TableProcessor(emgReference));
tracking.setReferenceLabel('/forceset/gasmed_l', 'gastrocnemius');
tracking.setReferenceLabel('/forceset/tibant_l', 'tibialis_anterior');
tracking.setReferenceLabel('/forceset/bfsh_l', 'biceps_femoris');
tracking.setReferenceLabel('/forceset/glmax2_l', 'gluteus');

% Part 3c: The EMG signals in the tracking are all normalized to have
% a maximum value of 1, but the magnitudes of the excitations from the 
% effort minimization solution suggest that these signals should be
% rescaled. Use addScaleFactor() to add a MocoParameter to the problem that
% will scale the reference data for the muscles in the tracking cost.
tracking.addScaleFactor('gastroc_factor', '/forceset/gasmed_l', [0.01 1.0]); 
tracking.addScaleFactor('tibant_factor', '/forceset/tibant_l', [0.01 1.0]); 
tracking.addScaleFactor('bifem_factor', '/forceset/bfsh_l', [0.01 1.0]); 
tracking.addScaleFactor('gluteus_factor', '/forceset/glmax2_l', [0.01 1.0]); 

% Part 3d: Add the tracking goal to the problem.
problem.addGoal(tracking)

% Part 3e: Update the MocoCasADiSolver with the updated MocoProblem using 
% resetProblem().
solver = MocoCasADiSolver.safeDownCast(study.updSolver());
solver.resetProblem(problem);

% Part 3f: Tell MocoCasADiSolver that the MocoParameters we added to the 
% problem via addScaleFactor() above do not require initSystem() calls on
% the model. This provides a large speed-up.
solver.set_parameters_require_initsystem(false);

if ~exist('trackingSolution.sto', 'file')
    % Part 3g: Solve the problem!
    solution = study.solve();
    solution.write('trackingSolution.sto');
end

% Part 3h: Get the values of the optimized scale factors.
trackingSolution = MocoTrajectory('trackingSolution.sto');
gastroc_factor = trackingSolution.getParameter('gastroc_factor');
tibant_factor = trackingSolution.getParameter('tibant_factor');
bifem_factor = trackingSolution.getParameter('bifem_factor');
gluteus_factor = trackingSolution.getParameter('gluteus_factor');

%% Part 4: Plot the EMG-tracking muscle redundancy problem solution.
% Part 4a: Print the scale factor values to the command window.
fprintf('\n')
fprintf('Optimized scale factor values: \n')
fprintf('------------------------------ \n')
fprintf(['gastrocnemius = ' num2str(gastroc_factor) '\n'])
fprintf(['tibialis anterior = ' num2str(tibant_factor) '\n'])
fprintf(['biceps femoris short head = ' num2str(bifem_factor) '\n'])
fprintf(['gluteus = ' num2str(gluteus_factor) '\n'])
fprintf('\n')

% Part 4b: Re-scale the reference data using the optimized scale factors.
gastroc = emgReference.updDependentColumn('gastrocnemius');
tibant = emgReference.updDependentColumn('tibialis_anterior');
bifem = emgReference.updDependentColumn('biceps_femoris');
gluteus = emgReference.updDependentColumn('gluteus');
for t = 0:emgReference.getNumRows() - 1
    gastroc.set(t, gastroc_factor * gastroc.get(t));
    tibant.set(t, tibant_factor * tibant.get(t));
    bifem.set(t, bifem_factor * bifem.get(t));
    gluteus.set(t, gluteus_factor * gluteus.get(t));
end

% Part 4c: Generate the plots. Compare results to the effort minimization 
% solution.
compareSolutionToEMG(emgReference, 'effortSolution.sto', ... 
    'trackingSolution.sto');

end

% Add a CoordinateActuator to the provided model with a specified optimal
% force. 
function addCoordinateActuator(model, coordinateName, optForce)

import org.opensim.modeling.*;

coordSet = model.getCoordinateSet();

actu = CoordinateActuator();
actu.setName(['torque_' coordinateName]);
actu.setCoordinate(coordSet.get(coordinateName));
actu.setOptimalForce(optForce);
actu.setMinControl(-1);
actu.setMaxControl(1);

% Add to ForceSet
model.addForce(actu);

end

function [modelProcessor] = getWalkingModel()

import org.opensim.modeling.*;

% Load the 19 DOF, 18 muscle model from file.
model = Model('subject_walk_armless_18musc.osim');

% Add actuators representing the pelvis residual actuators and lumbar
% torques. These actuators are only as strong as they need to be for the
% problem to converge.
addCoordinateActuator(model, 'pelvis_tx', 60)
addCoordinateActuator(model, 'pelvis_ty', 300)
addCoordinateActuator(model, 'pelvis_tz', 35)
addCoordinateActuator(model, 'pelvis_tilt', 60)
addCoordinateActuator(model, 'pelvis_list', 35)
addCoordinateActuator(model, 'pelvis_rotation', 25)
addCoordinateActuator(model, 'lumbar_bending', 40)
addCoordinateActuator(model, 'lumbar_extension', 40)
addCoordinateActuator(model, 'lumbar_rotation', 25)

% We need additional actuators for hip rotation and hip adduction since the
% existing muscle act primarily in the sagittal plane.
addCoordinateActuator(model, 'hip_rotation_r', 100)
addCoordinateActuator(model, 'hip_rotation_l', 100)
addCoordinateActuator(model, 'hip_adduction_r', 100)
addCoordinateActuator(model, 'hip_adduction_l', 100)

% Create a ModelProcessor to make additional modifications to the model.
modelProcessor = ModelProcessor(model);
% Weld the subtalar and toe joints.
jointsToWeld = StdVectorString();
jointsToWeld.add('subtalar_r');
jointsToWeld.add('subtalar_l');
jointsToWeld.add('mtp_r');
jointsToWeld.add('mtp_l');
modelProcessor.append(ModOpReplaceJointsWithWelds(jointsToWeld));
% Apply the ground reaction forces to the model.
modelProcessor.append(ModOpAddExternalLoads('external_loads.xml'));
% Update the muscles: ignore tendon compliance and passive forces, replace 
% muscle types with the DeGrooteFregly2016Muscle type, and scale the width
% of the active force length curve.
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
% Add a set a weak reserves to the sagittal-plane joints in the model.
modelProcessor.append(ModOpAddReserves(1.0));

end

function compareSolutionToEMG(varargin)

import org.opensim.modeling.*;

% Retrieve the inputs.
emgReference = varargin{1};
effortSolution = MocoTrajectory(varargin{2});
if nargin > 2
   trackingSolution = MocoTrajectory(varargin{3}); 
end

% Create a time vector for the EMG data that is consistent with the problem
% time range.
time = effortSolution.getTimeMat();
startIndex = emgReference.getNearestRowIndexForTime(time(1)) + 1;
endIndex = emgReference.getNearestRowIndexForTime(time(end)) + 1;
emgTime = linspace(time(1), time(end), endIndex - startIndex + 1);

% Plot results from the muscles in the left leg. 
titles = {'soleus', 'gastroc.', 'tib. ant.', 'hamstrings', ...
          'bi. fem. sh.', 'vastus', 'rec. fem.', 'gluteus', 'psoas'};
emgSignals = {'soleus',	'gastrocnemius', 'tibialis_anterior', ... 
              'hamstrings', 'biceps_femoris', 'vastus', ...
              'rectus_femoris', 'gluteus', 'none'};
muscles = {'soleus_l', 'gasmed_l', 'tibant_l', 'semimem_l', 'bfsh_l', ...
           'vasint_l', 'recfem_l', 'glmax2_l', 'psoas_l'};     
figure;
for i = 1:length(titles)
    subplot(3,3,i)
    
    % If it exists, plot the EMG signal for this muscle.
    if ~strcmp(emgSignals{i}, 'none')
        col = emgReference.getDependentColumn(emgSignals{i}).getAsMat();
        emg = col(startIndex:endIndex)';
        patch([emgTime fliplr(emgTime)], [emg fliplr(zeros(size(emg)))], ...
            [0.7 0.7 0.7])
        alpha(0.5);
    end
    hold on
    
    % Plot the control from the muscle effort solution.
    control = effortSolution.getControlMat(['/forceset/' muscles{i}]);
    plot(time, control, '-k', 'linewidth', 2)
    
    % If provided, plot the control from the tracking solution.
    if nargin > 2
        control = trackingSolution.getControlMat(['/forceset/' muscles{i}]);
        plot(time, control, '-r', 'linewidth', 2)
    end
    
    % Plot formatting.
    title(titles{i})
    xlim([0.83 2.0])
    ylim([0 1])
    if i == 1
        if nargin > 2
            legend('EMG', 'effort', 'tracking', 'location', 'best')
        else
            legend('EMG', 'effort', 'location', 'best')
        end
    end
    if i > 6
        xlabel('time (s)')
    else
        xticklabels([])
    end
    if ~mod(i-1, 3)
        ylabel('excitation')
    else
        yticklabels([])
    end
end


end
