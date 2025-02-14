# -------------------------------------------------------------------------- #
# OpenSim Moco: example3DWalking.py                                          #
# -------------------------------------------------------------------------- #
# Copyright (c) 2025 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Nicholas Bianco                                                 #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License"); you may    #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #

# This example demonstrates how to solve 3D walking optimization problems 
# using a foot-ground contact model. Polynomial functions are used to 
# represent muscle geometry via the FunctionBasedPath class which 
# significantly improves convergence time.
#
# See the README.txt next to this file for more information about the
# reference data used in this example.

import os
import opensim as osim

# Construct a MocoStudy to track joint kinematics and ground reaction forces
# using a torque-driven or muscle-driven model with foot-ground contact
# elements.
def runTrackingStudy(model, muscleDriven):

    # Paths to the contact forces in the model.
    contactForcesRight = osim.StdVectorString() 
    contactForcesRight.append('/contactHeel_r')
    contactForcesRight.append('/contactLateralRearfoot_r')
    contactForcesRight.append('/contactLateralMidfoot_r') 
    contactForcesRight.append('/contactMedialMidfoot_r')
    contactForcesRight.append('/contactLateralToe_r') 
    contactForcesRight.append('/contactMedialToe_r')

    contactForcesLeft = osim.StdVectorString()
    contactForcesLeft.append('/contactHeel_l')
    contactForcesLeft.append('/contactLateralRearfoot_l')
    contactForcesLeft.append('/contactLateralMidfoot_l') 
    contactForcesLeft.append('/contactMedialMidfoot_l')
    contactForcesLeft.append('/contactLateralToe_l')
    contactForcesLeft.append('/contactMedialToe_l')

    # Set the study name and weights. In the torque-driven problem, we 
    # choose weights to track the kinematics and ground reaction forces
    # more closely. In the muscle-driven problem, we reduce the tracking
    # weights slightly such that the muscles have a larger influence on
    # the optimized motion. The scale of the weights was chosen such
    # the optimized objective value falls roughly in the range [0.1, 10],
    # which generally improves convergence.
    if muscleDriven:
        studyName = 'muscle_driven_tracking'
        stateTrackingWeight = 0.05
        controlEffortWeight = 0.1
        contactTrackingWeight = 5e-3
    else:
        studyName = 'torque_driven_tracking'
        stateTrackingWeight = 0.1
        controlEffortWeight = 0.1
        contactTrackingWeight = 1e-2

    # Modify the model to prepare it for tracking optimization
    model.initSystem()
    modelProcessor = osim.ModelProcessor(model)
    if muscleDriven:
        modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
        modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
        modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
        modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
        modelProcessor.append(osim.ModOpReplacePathsWithFunctionBasedPaths( 
            'subject_walk_scaled_FunctionBasedPathSet.xml'))
    else:
        modelProcessor.append(osim.ModOpRemoveMuscles())
        modelProcessor.append(osim.ModOpAddReserves(500, 1.0, True, True))

    # Construct the reference kinematics TableProcessor.
    tableProcessor = osim.TableProcessor('coordinates.sto')
    tableProcessor.append(osim.TabOpUseAbsoluteStateNames())
    tableProcessor.append(osim.TabOpAppendCoupledCoordinateValues())
    tableProcessor.append(osim.TabOpAppendCoordinateValueDerivativesAsSpeeds())
    
    # Construct the MocoTrack problem.
    track = osim.MocoTrack()
    track.setName(studyName)
    track.setModel(modelProcessor)
    track.setStatesReference(tableProcessor)
    track.set_states_global_tracking_weight(stateTrackingWeight)
    track.set_control_effort_weight(controlEffortWeight)
    track.set_allow_unused_references(True)
    track.set_track_reference_position_derivatives(True)
    track.set_initial_time(0.48)
    track.set_final_time(1.61)
    track.set_mesh_interval(0.02)
    
    # Don't track the veritcal position of the pelvis and only lightly track
    # the speed. Let the optimization determine the vertical position of the
    # model, which will make it easier to find the position of the feet that 
    # leads to the best tracking of the kinematics and ground reaction forces.
    statesWeightSet = osim.MocoWeightSet()
    statesWeightSet.cloneAndAppend(
            osim.MocoWeight('/jointset/ground_pelvis/pelvis_ty/value', 0.0))
    statesWeightSet.cloneAndAppend(
            osim.MocoWeight('/jointset/ground_pelvis/pelvis_ty/speed', 0.1))
    track.set_states_weight_set(statesWeightSet)
    
    # Get the underlying MocoStudy.
    study = track.initialize()
    problem = study.updProblem()
    
    # Add a MocoContactTrackingGoal to the problem to track the ground 
    # reaction forces.
    contactTracking = osim.MocoContactTrackingGoal(
            'grf_tracking', contactTrackingWeight)
    contactTracking.setExternalLoadsFile('grf_walk.xml')
    toeBodyRight = osim.StdVectorString()
    toeBodyRight.append('/bodyset/toes_r')
    contactTracking.addContactGroup(osim.MocoContactTrackingGoalGroup(
            contactForcesRight, 'Right_GRF', toeBodyRight))
    toeBodyLeft = osim.StdVectorString()
    toeBodyLeft.append('/bodyset/toes_l')
    contactTracking.addContactGroup(osim.MocoContactTrackingGoalGroup(
            contactForcesLeft, 'Left_GRF', toeBodyLeft))
    problem.addGoal(contactTracking)

    # Constrain the initial states to be close to the reference.
    coordinatesUpdated = tableProcessor.process(model)
    labels = coordinatesUpdated.getColumnLabels()
    index = coordinatesUpdated.getNearestRowIndexForTime(0.48)
    for label in labels:
        value = coordinatesUpdated.getDependentColumn(label).to_numpy()
        if '/speed' in label:
            lower = value[index] - 0.1
            upper = value[index] + 0.1
        else:
            lower = value[index] - 0.05
            upper = value[index] + 0.05
        problem.setStateInfo(label, [], [lower, upper])

    # Constrain the states and controls to be periodic.
    if muscleDriven:
        periodicityGoal = osim.MocoPeriodicityGoal('periodicity')
        coordinates = model.getCoordinateSet()
        for icoord in range(coordinates.getSize()):
            coordinate = coordinates.get(icoord)
            coordName = coordinate.getName()

            # Exclude the knee_angle_l/r_beta coordinates from the periodicity
            # constraint because they are coupled to the knee_angle_l/r
            # coordinates.
            if 'beta' in coordName: continue 
    
            if not '_tx' in coordName:
                valueName = coordinate.getStateVariableNames().get(0)
                periodicityGoal.addStatePair(
                        osim.MocoPeriodicityGoalPair(valueName))
            speedName = coordinate.getStateVariableNames().get(1)
            periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair(speedName))

        muscles = model.getMuscles()
        for imusc in range(muscles.getSize()):
            muscle = muscles.get(imusc)
            stateName = muscle.getStateVariableNames().get(0)
            periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair(stateName))
            controlName = muscle.getAbsolutePathString()
            periodicityGoal.addControlPair(
                    osim.MocoPeriodicityGoalPair(controlName))

        actuators = model.getActuators()
        for iactu in range(actuators.getSize()):
            actu = osim.CoordinateActuator.safeDownCast(actuators.get(iactu))
            if actu is not None: 
                controlName = actu.getAbsolutePathString()
                periodicityGoal.addControlPair(
                        osim.MocoPeriodicityGoalPair(controlName))
        
        problem.addGoal(periodicityGoal)
    
    # Customize the solver settings.
    # ------------------------------
    solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
    # Use the Legendre-Gauss-Radau transcription scheme, a psuedospectral 
    # scheme with high integration accuracy.
    solver.set_transcription_scheme('legendre-gauss-radau-3')
    # Use the Bordalba et al. (2023) kinematic constraint method.
    solver.set_kinematic_constraint_method('Bordalba2023')
    # Set the solver's convergence and constraint tolerances.
    solver.set_optim_convergence_tolerance(1e-2)
    solver.set_optim_constraint_tolerance(1e-4)
    # We've updated the MocoProblem, so call resetProblem() to pass the updated
    # problem to the solver.
    solver.resetProblem(problem)
    # When MocoTrack::initialize() is called, the solver is created with a
    # default guess. Since we've updated the problem and changed the
    # transcription scheme, it is a good idea to generate a new guess. If
    # solving a muscle-driven problem, use the solution from the 
    # torque-driven problem as the initial guess.
    guess = solver.createGuess()
    torqueDrivenSolutionFile = \
            'example3DWalking_torque_driven_tracking_solution.sto'
    if muscleDriven and os.path.exists(torqueDrivenSolutionFile):
        initialGuess = osim.MocoTrajectory(torqueDrivenSolutionFile)
        guess.insertStatesTrajectory(initialGuess.exportToStatesTable(), True)
        controls = guess.exportToControlsTable()
        controls.updMatrix().setToZero()
        guess.insertControlsTrajectory(controls, True)
    solver.setGuess(guess)
    
    # Solve!
    # ------
    solution = study.solve()
    solution.write(f'example3DWalking_{studyName}_solution.sto')
    
    # Print the model.
    modelSolution = modelProcessor.process()
    modelSolution.initSystem()
    modelSolution.printToXML(f'example3DWalking_{studyName}_model.osim')
    
    # Extract the ground reaction forces.
    externalForcesTableFlat = osim.createExternalLoadsTableForGait(
            modelSolution, solution, contactForcesRight, contactForcesLeft)
    osim.STOFileAdapter.write(externalForcesTableFlat, 
            f'example3DWalking_{studyName}_ground_reactions.sto')
    
    # Visualize the solution.
    study.visualize(solution)


# Model preparation.
# -----------------
# Update the model to prepare it for tracking optimization. The default 
# minimimum muscle excitations and activations are set to 0 stiffness, 
# damping, and light torque actuation are added to the toes and contact 
# geometry is added to the foot bodies in the model. The height of the 
# contact geometry elements are adjusted to better align with the ground.
model = osim.Model('subject_walk_scaled.osim')
model.initSystem()

# Set minimum muscle controls and activations to 0 (default is 0.01).
muscles = model.updMuscles()
for imuscle in range(muscles.getSize()):
    muscle = osim.Millard2012EquilibriumMuscle.safeDownCast(muscles.get(imuscle))
    muscle.setMinimumActivation(0.0)
    muscle.setMinControl(0.0)

# Add stiffness and damping to the toes. Based on Falisse et al. (2022), 
# "Modeling toes contributes to realistic stance knee mechanics in 
# three-dimensional predictive simulations of walking."
ebcf_toes_l = osim.ExpressionBasedCoordinateForce(
        'mtp_angle_l', '-25.0*q-2.0*qdot')
ebcf_toes_l.setName('toe_damping_l')
model.addForce(ebcf_toes_l)
ebcf_toes_r = osim.ExpressionBasedCoordinateForce(
        'mtp_angle_r', '-25.0*q-2.0*qdot')
ebcf_toes_r.setName('toe_damping_r')
model.addForce(ebcf_toes_r)

# Add relatively strong CoordinateActuators to the toes, since no muscles
# actuate the toes in this example.
ca_toes_l = osim.CoordinateActuator('mtp_angle_l')
ca_toes_l.setName('mtp_angle_l_actuator')
ca_toes_l.setOptimalForce(50)
ca_toes_l.setMinControl(-1.0)
ca_toes_l.setMaxControl(1.0)
model.addForce(ca_toes_l)

ca_toes_r = osim.CoordinateActuator('mtp_angle_r')
ca_toes_r.setName('mtp_angle_r_actuator')
ca_toes_r.setOptimalForce(50)
ca_toes_r.setMinControl(-1.0)
ca_toes_r.setMaxControl(1.0)
model.addForce(ca_toes_r)

# Add the contact geometry to the model.
contactGeometrySet = osim.ContactGeometrySet(
        'subject_walk_scaled_ContactGeometrySet.xml')
for i in range(contactGeometrySet.getSize()):
    contactGeometry = contactGeometrySet.get(i).clone()
    # Raise the ContactSpheres by 2 cm so that bottom of the spheres
    # are better aligned with the ground.
    if 'floor' not in contactGeometry.getName():
        location = contactGeometry.upd_location()
        location.set(1, location.get(1) + 0.02)
    model.addContactGeometry(contactGeometry)

# Add the contact forces to the model.
contactForceSet = osim.ForceSet('subject_walk_scaled_ContactForceSet.xml')
for i in range(contactForceSet.getSize()):
    model.addComponent(contactForceSet.get(i).clone())
model.finalizeConnections()

# Tracking optimization.
# ---------------------
# Solve tracking optimization problems using the modified model. The
# convergence times below were estimated on a machine using a processor
# with 4.7 GHz base clock speed and 12 CPU cores (12 threads).

# Solve a torque-driven tracking problem to create a kinematic 
# trajectory that is consistent with the ground reaction forces.
# This problem takes ~10 minutes to solve.
runTrackingStudy(model, False)

# Solve a muscle-driven tracking problem using the kinematic trajectory
# from the torque-driven problem as the initial guess.
# This problem takes ~100 minutes to solve.
runTrackingStudy(model, True)
