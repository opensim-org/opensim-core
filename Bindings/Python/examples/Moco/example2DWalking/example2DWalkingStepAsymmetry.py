# ---------------------------------------------------------------------------- #
# OpenSim Moco: example2DWalkingStepAsymmetry.py                               #
# ---------------------------------------------------------------------------- #
# Copyright (c) 2021 Stanford University and the Authors                       #
#                                                                              #
# Author(s): Russell T. Johnson                                                #
#            University of Southern California, rtjohnso@usc.edu               # 
#                                                                              #
# Licensed under the Apache License, Version 2.0 (the "License"); you may      #
# not use this file except in compliance with the License. You may obtain a    #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0            #
#                                                                              #
# Unless required by applicable law or agreed to in writing, software          #
# distributed under the License is distributed on an "AS IS" BASIS,            #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.     #
# See the License for the specific language governing permissions and          #
# limitations under the License.                                               #
# ---------------------------------------------------------------------------- #

import os
import opensim as osim
import re
import math
import sys
import numpy as np

def findHeelStrikeIndex(verticalGRF, forceThreshold):

    contactIndices = np.where(verticalGRF > forceThreshold)
    nonContactIndices = np.where(verticalGRF < forceThreshold)

    if nonContactIndices[0] > 0:
        index = nonContactIndices[:-1] + 1
    else:
        index = contactIndices[0]

    if index > len(verticalGRF):
        index = 0

    return index

def computeStepLength(model, state):

    model.initSystem()
    model.realizePosition(state)

    leftContactGeometry = model.getContactGeometrySet.get("heel_r")
    rightContactGeometry = model.getContactGeometrySet.get("heel_l")

    rightHeelPosition = leftContactGeometry.getFrame().getPositionInGround(state)
    leftHeelPosition = rightContactGeometry.getFrame().getPositionInGround(state)

    stepLength = abs(rightHeelPosition.get(0) - leftHeelPosition.get(0))

    return stepLength

def computeStepAsymmetryValues(solutionFile, grfsFile):

    model = osim.Model("2D_gait.osim")
    solution = osim.TimeSeriesTable(solutionFile)
    grfs = osim.TimeSeriesTable(grfsFile)

    # Get time vector
    nrow = grfs.getNumRows()
    timeVec = grfs.getIndependentColumn()
    time = np.zeros((nrow, ))
    for i in range(nrow):
        time[i] = timeVec.get(i)

    # Find the time of the left and right heelstrikes
    contactForceThreshold = 25 # N
    rightVerticalGRF = grfs.getDependentColumn("ground_force_r_vy").getAsMat()
    leftVerticalGRF = grfs.getDependentColumn("ground_force_l_vy").getAsMat()
    rightHeelStrikeIndex = findHeelStrikeIndex(rightVerticalGRF, 
                                               contactForceThreshold)
    leftHeelStrikeIndex = findHeelStrikeIndex(leftVerticalGRF, 
                                              contactForceThreshold)
    rightHeelStrike = time[rightHeelStrikeIndex]
    leftHeelStrike = time[leftHeelStrikeIndex]

    # Compute step time asymmetry
    if rightHeelStrike < leftHeelStrike:
        leftStepTime = leftHeelStrike - rightHeelStrike
        rightStepTime = time[:-1] - leftHeelStrike + rightHeelStrike
    else: 
        rightStepTime = rightHeelStrike - leftHeelStrike
        leftStepTime = time[:-1] - rightHeelStrike + leftHeelStrike
    stepTimeAsymmetry = ((rightStepTime - leftStepTime) / ... 
                        (rightStepTime + leftStepTime)) * 100.0

    # Create StatesTrajectory from solution             
    statesTraj = osim.StatesTrajectory().createFromStatesTable(model, solution, 
                                                               False, True, True)

    stateRHS = statesTraj.get(rightHeelStrikeIndex)
    stateLHS = statesTraj.get(leftHeelStrikeIndex)
                    
    rightStepLength = computeStepLength(model, stateRHS)
    leftStepLength = computeStepLength(model, stateLHS)

    stepLengthAsymmetry = ((rightStepLength - leftStepLength) / ...
                        (rightStepLength + leftStepLength)) * 100.0
    
    return stepTimeAsymmetry, stepLengthAsymmetry

# Step Time Asymmetry
# -------------------
# Set up a predictive optimization problem where the goal is to minimize an 
# effort cost (cubed controls) and hit a target step time asymmetry. Unlike 
# example2DWalking, this problem requires simulating a full gait cycle. 
# Additionally, endpoint constraints enforce periodicity of the coordinate values 
# (except for pelvis tx) and speeds, coordinate actuator controls, and muscle 
# activations.
#
# Step time is defined as the time between consecutive foot strikes. Step Time 
# Asymmetry (STA) is a ratio and is calculated as follows:
#  - Right Step Time (RST) = Time from left foot-strike to right foot-strike
#  - Left Step Time (LST)  = Time from right foot-strike to left foot-strike
#  - STA = (RST - LST) / (RST + LST)
#
# The step time goal works by "counting" the number of nodes that each foot is in 
# contact with the ground (with respect to a specified contact force threshold). 
# Since, in walking, there are double support phases where both feet are on the 
# ground, the goal also detects which foot is in front and assigns the step time 
# to the leading foot. Altogether, it estimates the time between consecutive 
# heel strikes in order to infer the left and right step times.
#
# The contact elements for each foot must specified via 'setLeftContactGroup()'
# and 'setRightContactGroup()'. The force element and force threshold used to 
# determine when a foot is in contact is set via 'setContactForceDirection()' and 
# 'setContactForceThreshold()'.
#
# Users must provide the target asymmetry value via 'setTargetAsymmetry()'.
# Asymmetry values ranges from -1.0 to 1.0. For example, 0.20 is 20# positive
# step time asymmetry with greater right step times than left step times. A
# symmetric step times solution can be achieved by setting this property to zero.
# This goal can be used only in 'cost' mode, where the error between the target
# asymmetry and model asymmetry is squared. To make this goal suitable for
# gradient-based optimization, step time values are assigned via smoothing
# functions which can be controlled via 'setAsymmetrySmoothing()' and
# 'setContactDetectionSmoothing()'.
#
# Since this goal doesn't directly compute the step time asymmetry from heel 
# strikes, users should confirm that the step time asymmetry from the solution 
# matches closely to the target. To do this, we provide the helper function 
# computeStepAsymmetryValues() below.
def stepTimeAsymmetry():

    # Create a MocoStudy
    # ------------------
    study = osim.MocoStudy()
    study.setName("example2DWalking_StepTimeAsymmetry")

    # Define the MocoProblem
    # ----------------------
    problem = study.updProblem()
    modelProcessor = osim.ModelProcessor("2D_gait.osim")                   
    modelProcessor.append(osim.ModOpTendonComplianceDynamicsModeDGF("implicit")) 
    problem.setModelProcessor(modelProcessor)  
    problem.setTimeBounds(0, 0.94)

    # Goals
    # =====

    # Periodicity 
    # -----------
    periodicityGoal = osim.MocoPeriodicityGoal("periodicity")
    model = modelProcessor.process()
    model.initSystem()
    # All states are periodic except for the value of the pelvis_tx coordinate.
    for i in range (model.getNumStateVariables()):
        currentStateName = model.getStateVariableNames().getitem(i)
        if not currentStateName.__contains__("pelvis_tx/value"):
            periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair(currentStateName))

    # The lumbar actuator control is periodic.
    periodicityGoal.addControlPair(osim.MocoPeriodicityGoalPair("/lumbarAct"))
    problem.addGoal(periodicityGoal)

    # Average gait speed
    # ------------------
    speedGoal = osim.MocoAverageSpeedGoal("speed")
    speedGoal.set_desired_average_speed(1.0)
    problem.addGoal(speedGoal)

    # Effort over distance
    # --------------------
    effortGoal = osim.MocoControlGoal("effort", 10.0)
    effortGoal.setExponent(3)
    effortGoal.setDivideByDisplacement(True)
    problem.addGoal(effortGoal)

    # Step time asymmetry
    # -------------------
    # The settings here have been modified from the default values to suit this 
    # specific problem.
    stepTimeAsymmetry = osim.MocoStepTimeAsymmetryGoal()
    # Value for smoothing term used to compute when foot contact is made (default is 
    # 0.25). Users may need to adjust this based on convergence and matching the 
    # target asymmetry.
    stepTimeAsymmetry.setContactDetectionSmoothing(0.4)
    # (N) contact threshold based on vertical GRF; default value = 25
    stepTimeAsymmetry.setContactForceThreshold(25)
    # Value for smoothing term use to compute asymmetry (default is 10). Users may 
    # need to adjust this based on convergence and matching the target
    # asymmetry.
    stepTimeAsymmetry.setAsymmetrySmoothing(3)
    # Target step length asymmetry: positive numbers mean greater right step lengths 
    # than left.
    stepTimeAsymmetry.setTargetAsymmetry(0.10)   
    # Set goal weight.
    stepTimeAsymmetry.setWeight(5)          

    # Need to define the names of the left and right heel spheres: this is
    # used to detect which foot is in front during double support phase.
    forceNamesRightFoot = osim.StdVectorString()
    forceNamesRightFoot.append("contactHeel_r")
    forceNamesRightFoot.append("contactFront_r")
    forceNamesLeftFoot = osim.StdVectorString()
    forceNamesLeftFoot.append("contactHeel_l")
    forceNamesLeftFoot.append("contactFront_l")
    stepTimeAsymmetry.setRightContactGroup(forceNamesRightFoot, "contactHeel_r")
    stepTimeAsymmetry.setLeftContactGroup(forceNamesLeftFoot, "contactHeel_l")

    # Add the goal to the problem. 
    problem.addGoal(stepTimeAsymmetry)

    # Bounds
    # ======
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value", [-20*math.pi/180, 20*math.pi/180])
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value",[0, 2], 0)
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value", [0.75, 1.25])
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value", [-10*math.pi/180, 60*math.pi/180])
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value", [-10*math.pi/180, 60*math.pi/180])
    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value", [-50*math.pi/180, 0])
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value", [-50*math.pi/180, 0])
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value", [-15*math.pi/180, 25*math.pi/180])
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value", [-15*math.pi/180, 25*math.pi/180])
    problem.setStateInfo("/jointset/lumbar/lumbar/value", [0, 20*math.pi/180])

    # Configure the solver
    # ====================
    solver = study.initCasADiSolver()
    solver.set_num_mesh_intervals(100)
    solver.set_verbosity(2)
    solver.set_optim_convergence_tolerance(1e-4)
    solver.set_optim_constraint_tolerance(1e-4)
    solver.set_optim_max_iterations(2000)

    # Use the tracking problem solution from example2DWalking as the initial
    # guess, if it exists. If it doesn't exist, users can run example2DWalking.m to 
    # generate this file.
    if os.path.exists("gaitTracking_solution_fullStride.sto"):
        solver.setGuessFile("gaitTracking_solution_fullStride.sto")

    # Now that we've finished setting up the MocoStudy, print it to a file.
    study.printToXML("example2DWalking_StepTimeAsymmetry.omoco")

    # Solve the problem
    # =================
    solution = study.solve()

    # Write the solution to a file.
    solution.write("example2DWalking_StepTimeAsymmetry_solution.sto")

    # Write solution's GRF to a file.
    externalForcesTableFlat = osim.createExternalLoadsTableForGait(model, 
                                                                   solution, forceNamesRightFoot, forceNamesLeftFoot)
    osim.STOFileAdapter.write(externalForcesTableFlat, 
                              "example2DWalking_StepTimeAsymmetry_grfs.sto")
    
    # Compute the actual step time asymmetry.
    stepTimeAsymmetry,_ = computeStepAsymmetryValues(
        "example2DWalking_StepTimeAsymmetry_solution.sto",
          "example2DWalking_StepTimeAsymmetry_grfs.sto")
    
    print("\n")
    print(f"Step Time Asymmetry = {stepTimeAsymmetry:.1f}%")
    print("\n")

    # Visualize solution.
    study.visualize(solution)

def example2DWalkingStepAsymmetry():

    # Simulate asymmetric step times using MocoStepTimeAsymmetryGoal.
    stepTimeAsymmetry()

    # Simulate asymmetric step lengths using MocoStepLengthAsymmetryGoal.
    #stepLengthAsymmetry()

example2DWalkingStepAsymmetry()
