% -------------------------------------------------------------------------- %
% OpenSim Moco: example3DWalking.m                                           %
% -------------------------------------------------------------------------- %
% Copyright (c) 2025 Stanford University and the Authors                     %
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

% This example demonstrates how to solve 3D walking optimization problems 
% using a foot-ground contact model. Polynomial functions are used to 
% represent muscle geometry via the FunctionBasedPath class which 
% significantly improves convergence time.
%
% See the README.txt next to this file for more information about the
% reference data used in this example.

function example3DWalking()

    % Import the OpenSim libraries.
    import org.opensim.modeling.*;

    % Model preparation.
    % -----------------
    % Update the model to prepare it for tracking optimization. The default 
    % minimimum muscle excitations and activations are set to 0; stiffness, 
    % damping, and light torque actuation are added to the toes; and contact 
    % geometry is added to the foot bodies in the model. The height of the 
    % contact geometry elements are adjusted to better align with the ground.
    model = Model('subject_walk_scaled.osim');
    model.initSystem();

    % Set minimum muscle controls and activations to 0 (default is 0.01).
    muscles = model.updMuscles();
    for imuscle = 1:muscles.getSize()
        muscle = Millard2012EquilibriumMuscle.safeDownCast(...
            muscles.get(imuscle-1));
        muscle.setMinimumActivation(0.0);
        muscle.setMinControl(0.0);
    end

    % Add stiffness and damping to the joints. Toe stiffness and damping values
    % are based on Falisse et al. (2022), "Modeling toes contributes to 
    % realistic stance knee mechanics in three-dimensional predictive 
    % simulations of walking."
    expressionBasedForceSet = ForceSet(...
            'subject_walk_scaled_ExpressionBasedCoordinateForceSet.xml');
    for i = 0:expressionBasedForceSet.getSize()-1
        model.addComponent(expressionBasedForceSet.get(i).clone());
    end

    % Add the contact geometry to the model.
    contactGeometrySet = ContactGeometrySet(...
            'subject_walk_scaled_ContactGeometrySet.xml');
    for i = 0:contactGeometrySet.getSize()-1
        contactGeometry = contactGeometrySet.get(i).clone();
        % Raise the ContactSpheres by 2 cm so that bottom of the spheres
        % are better aligned with the ground.
        if ~strcmp(contactGeometry.getName(), 'floor')
            location = contactGeometry.upd_location();
            location.set(1, location.get(1) + 0.02);
        end
        model.addContactGeometry(contactGeometry);
    end

    % Add the contact forces to the model.
    contactForceSet = ForceSet('subject_walk_scaled_ContactForceSet.xml');
    for i = 0:contactForceSet.getSize()-1
        model.addComponent(contactForceSet.get(i).clone());
    end
    model.finalizeConnections();

    % Tracking optimization.
    % ---------------------
    % Solve tracking optimization problems using the modified model. The
    % convergence times below were estimated on a machine using a processor
    % with 4.7 GHz base clock speed and 12 CPU cores (12 threads).

    % Solve a torque-driven tracking problem to create a kinematic 
    % trajectory that is consistent with the ground reaction forces.
    % This problem takes ~10 minutes to solve.
    runTrackingStudy(model, false);

    % Solve a muscle-driven tracking problem using the kinematic trajectory
    % from the torque-driven problem as the initial guess.
    % This problem takes ~35 minutes to solve.
    runTrackingStudy(model, true);
end

% Construct a MocoStudy to track joint kinematics and ground reaction forces
% using a torque-driven or muscle-driven model with foot-ground contact
% elements. The objective function weights were chosen such that the optimized 
% objective value falls roughly in the range [0.1, 10], which generally 
% improves convergence.
function runTrackingStudy(model, muscleDriven)

    % Import the OpenSim libraries.
    import org.opensim.modeling.*;

    % Paths to the contact forces in the model.
    contactForcesRight = StdVectorString(); 
    contactForcesRight.add('/contactHeel_r');
    contactForcesRight.add('/contactLateralRearfoot_r');
    contactForcesRight.add('/contactLateralMidfoot_r'); 
    contactForcesRight.add('/contactMedialMidfoot_r');
    contactForcesRight.add('/contactLateralToe_r'); 
    contactForcesRight.add('/contactMedialToe_r');

    contactForcesLeft = StdVectorString();
    contactForcesLeft.add('/contactHeel_l');
    contactForcesLeft.add('/contactLateralRearfoot_l');
    contactForcesLeft.add('/contactLateralMidfoot_l'); 
    contactForcesLeft.add('/contactMedialMidfoot_l');
    contactForcesLeft.add('/contactLateralToe_l');
    contactForcesLeft.add('/contactMedialToe_l');

    % Configure study-specific settings.
    if (muscleDriven)
        studyName = 'muscle_driven_tracking';
    else
        studyName = 'torque_driven_tracking';

        % Add weak CoordinateActuators to the toes. For the torque-driven 
        % simulation, we do not want ModOpAddReserves() to add strong 
        % actuators to the toes, since the toes will have no active actuation
        % in the muscle-driven problem.
        ca_toes_l = CoordinateActuator('mtp_angle_l');
        ca_toes_l.setName('mtp_angle_l_actuator');
        ca_toes_l.setOptimalForce(10);
        ca_toes_l.setMinControl(-1.0);
        ca_toes_l.setMaxControl(1.0);
        model.addForce(ca_toes_l);

        ca_toes_r = CoordinateActuator('mtp_angle_r');
        ca_toes_r.setName('mtp_angle_r_actuator');
        ca_toes_r.setOptimalForce(10);
        ca_toes_r.setMinControl(-1.0);
        ca_toes_r.setMaxControl(1.0);
        model.addForce(ca_toes_r);
    end

    % Modify the model to prepare it for tracking optimization
    model.initSystem();
    modelProcessor = ModelProcessor(model);
    if (muscleDriven) 
        modelProcessor.append(ModOpIgnoreTendonCompliance());
        modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
        modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
        modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
        modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths( ...
            'subject_walk_scaled_FunctionBasedPathSet.xml'));
    else
        modelProcessor.append(ModOpRemoveMuscles());
        modelProcessor.append(ModOpAddReserves(500, 1.0, true, true));
    end

    % Construct the reference kinematics TableProcessor.
    tableProcessor = TableProcessor('coordinates.sto');
    tableProcessor.append(TabOpUseAbsoluteStateNames());
    tableProcessor.append(TabOpAppendCoupledCoordinateValues());
    tableProcessor.append(TabOpAppendCoordinateValueDerivativesAsSpeeds());
    
    % Construct the MocoTrack problem.
    track = MocoTrack();
    track.setName(studyName);
    track.setModel(modelProcessor);
    track.setStatesReference(tableProcessor);
    track.set_states_global_tracking_weight(0.05);
    track.set_control_effort_weight(0.1);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    track.set_initial_time(0.48);
    track.set_final_time(1.61);
    track.set_mesh_interval(0.02);
    
    % Don't track the veritcal position of the pelvis and only lightly track
    % the speed. Let the optimization determine the vertical position of the
    % model, which will make it easier to find the position of the feet that 
    % leads to the best tracking of the kinematics and ground reaction forces.
    statesWeightSet = MocoWeightSet();
    statesWeightSet.cloneAndAppend(...
            MocoWeight('/jointset/ground_pelvis/pelvis_ty/value', 0.0));
    statesWeightSet.cloneAndAppend(...
            MocoWeight('/jointset/ground_pelvis/pelvis_ty/speed', 0.1));
    track.set_states_weight_set(statesWeightSet);
    
    % Get the underlying MocoStudy.
    study = track.initialize();
    problem = study.updProblem();

    % Set bounds on the toe coordinate states.
    problem.setStateInfoPattern('/jointset/mtp_.*/value', [-1.0, 1.0]);
    problem.setStateInfoPattern('/jointset/mtp_.*/speed', [-20.0, 20.0]);
    
    % Add a MocoContactTrackingGoal to the problem to track the ground 
    % reaction forces.
    contactTracking = MocoContactTrackingGoal(...
            'grf_tracking', 5e-3);
    contactTracking.setExternalLoadsFile('grf_walk.xml');
    toeBodyRight = StdVectorString();
    toeBodyRight.add('/bodyset/toes_r');
    contactTracking.addContactGroup(MocoContactTrackingGoalGroup(...
            contactForcesRight, 'Right_GRF', toeBodyRight));
    toeBodyLeft = StdVectorString();
    toeBodyLeft.add('/bodyset/toes_l');
    contactTracking.addContactGroup(MocoContactTrackingGoalGroup(...
            contactForcesLeft, 'Left_GRF', toeBodyLeft));
    problem.addGoal(contactTracking);

    % Constrain the initial states to be close to the reference.
    coordinatesUpdated = tableProcessor.process(model);
    labels = coordinatesUpdated.getColumnLabels();
    index = coordinatesUpdated.getNearestRowIndexForTime(0.48);
    for i = 1:labels.size()
        label = string(labels.get(i-1));
        value = coordinatesUpdated.getDependentColumn(label);
        if (contains(label, '/speed'))
            lower = value.get(index) - 0.1;
            upper = value.get(index) + 0.1;
        else
            lower = value.get(index) - 0.05;
            upper = value.get(index) + 0.05;
        end
        problem.setStateInfo(label, [], [lower, upper]);
    end

    % Constrain the states and controls to be periodic.
    if (muscleDriven)
        periodicityGoal = MocoPeriodicityGoal('periodicity');
        coordinates = model.getCoordinateSet();
        for icoord = 1:coordinates.getSize()
            coordinate = coordinates.get(icoord-1);
            coordName = string(coordinate.getName());
            % Exclude the knee_angle_l/r_beta coordinates from the periodicity
            % constraint because they are coupled to the knee_angle_l/r
            % coordinates.
            if (contains(coordName, 'beta')) 
                continue; 
            end
            if (~contains(coordName, '_tx')) 
                valueName = coordinate.getStateVariableNames().get(0);
                periodicityGoal.addStatePair(...
                        MocoPeriodicityGoalPair(valueName));
            end
            speedName = coordinate.getStateVariableNames().get(1);
            periodicityGoal.addStatePair(MocoPeriodicityGoalPair(speedName));
        end

        muscles = model.getMuscles();
        for imusc = 1:muscles.getSize()
            muscle = muscles.get(imusc-1);
            stateName = muscle.getStateVariableNames().get(0);
            periodicityGoal.addStatePair(MocoPeriodicityGoalPair(stateName));
            controlName = muscle.getAbsolutePathString();
            periodicityGoal.addControlPair(...
                    MocoPeriodicityGoalPair(controlName));
        end

        actuators = model.getActuators();
        for iactu = 1:actuators.getSize()
            actu = CoordinateActuator.safeDownCast(actuators.get(iactu-1));
            if ~isempty(actu) 
                controlName = actu.getAbsolutePathString();
                periodicityGoal.addControlPair(...
                        MocoPeriodicityGoalPair(controlName));
            end
        end
        
        problem.addGoal(periodicityGoal);
    end
    
    % Customize the solver settings.
    % ------------------------------
    solver = MocoCasADiSolver.safeDownCast(study.updSolver());
    % Use the Legendre-Gauss-Radau transcription scheme, a psuedospectral 
    % scheme with high integration accuracy.
    solver.set_transcription_scheme('legendre-gauss-radau-3');
    % Use the Bordalba et al. (2023) kinematic constraint method.
    solver.set_kinematic_constraint_method('Bordalba2023');
    % Set the solver's convergence and constraint tolerances.
    solver.set_optim_convergence_tolerance(1e-2);
    solver.set_optim_constraint_tolerance(1e-4);
    % We've updated the MocoProblem, so call resetProblem() to pass the updated
    % problem to the solver.
    solver.resetProblem(problem);
    % When MocoTrack::initialize() is called, the solver is created with a
    % default guess. Since we've updated the problem and changed the
    % transcription scheme, it is a good idea to generate a new guess. If
    % solving a muscle-driven problem, use the solution from the 
    % torque-driven problem as the initial guess.
    guess = solver.createGuess();
    torqueDrivenSolutionFile = ...
            'example3DWalking_torque_driven_tracking_solution.sto';
    if (muscleDriven && exist(torqueDrivenSolutionFile, 'file'))
        initialGuess = MocoTrajectory(torqueDrivenSolutionFile);
        guess.insertStatesTrajectory(initialGuess.exportToStatesTable(), true);
        controls = guess.exportToControlsTable();
        controls.updMatrix().setToZero();
        guess.insertControlsTrajectory(controls, true);
    end
    solver.setGuess(guess);
    
    % Solve!
    % ------
    solution = study.solve();
    solution.write(sprintf('example3DWalking_%s_solution.sto', studyName));
    
    % Print the model.
    modelSolution = modelProcessor.process();
    modelSolution.initSystem();
    modelSolution.print(sprintf('example3DWalking_%s_model.osim', studyName));
    
    % Extract the ground reaction forces.
    externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(...
            modelSolution, solution, contactForcesRight, contactForcesLeft);
    STOFileAdapter.write(externalForcesTableFlat, ...
            sprintf('example3DWalking_%s_ground_reactions.sto', studyName));
    
    % Visualize the solution.
    study.visualize(solution);
end
