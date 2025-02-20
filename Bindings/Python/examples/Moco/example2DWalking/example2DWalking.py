# -------------------------------------------------------------------------- #
# OpenSim Moco: example2DWalking.py                                          #
# -------------------------------------------------------------------------- #
# Copyright (c) 2019 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Brian Umberger                                                  #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License") you may     #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #

# This is a Matlab implementation of an example optimal control
# problem (2-D walking) orginally created in C++ by Antoine Falisse
# (see: example2DWalking.cpp).
#
# This example features two different optimal control problems:
#  - The first problem is a tracking simulation of walking.
#  - The second problem is a predictive simulation of walking.
#
# The code is inspired from Falisse A, Serrancoli G, Dembia C, Gillis J,
# De Groote F: Algorithmic differentiation improves the computational
# efficiency of OpenSim-based trajectory optimization of human movement.
# PLOS One, 2019.
#
# Model
# -----
# The model described in the file '2D_gait.osim' included in this file is a
# modified version of the 'gait10dof18musc.osim' available within OpenSim. We
# replaced the moving knee flexion axis by a fixed flexion axis, replaced the
# Millard2012EquilibriumMuscles by DeGrooteFregly2016Muscles, and added
# SmoothSphereHalfSpaceForces (two contacts per foot) to model the
# contact interactions between the feet and the ground.
#
# Do not use this model for research. The path of the gastroc muscle contains
# an error--the path does not cross the knee joint.
#
# Data
# ----
# The coordinate data included in the 'referenceCoordinates.sto' comes from
# predictive simulations generated in Falisse et al. 2019.  As such,
# they deviate slightly from typical experimental gait data.

import os
import opensim as osim
import re
import math
import sys

# ---------------------------------------------------------------------------
# Set up a coordinate tracking problem where the goal is to minimize the
# difference between provided and simulated coordinate values and speeds (and
# ground reaction forces), as well as to minimize an effort cost (squared
# controls). The provided data represents half a gait cycle. Endpoint
# constraints enforce periodicity of the coordinate values (except for
# pelvis tx) and speeds, coordinate actuator controls, and muscle activations.


# Define the optimal control problem
# ==================================
track = osim.MocoTrack()
track.setName("gaitTracking")

# Set the weights for the terms in the objective function. The values below were
# obtained by trial and error.

# Note: If GRFTrackingWeight is set to 0 then GRFs will not be tracked. Setting
# GRFTrackingWeight to 1 will cause the total tracking error (states + GRF) to
# have about the same magnitude as control effort in the final objective value.
controlEffortWeight = 10
stateTrackingWeight = 1
GRFTrackingWeight   = 1


# Reference data for tracking problem
tableProcessor = osim.TableProcessor("referenceCoordinates.sto")
tableProcessor.append(osim.TabOpLowPassFilter(6))

modelProcessor = osim.ModelProcessor("2D_gait.osim")
track.setModel(modelProcessor)
track.setStatesReference(tableProcessor)
track.set_states_global_tracking_weight(stateTrackingWeight)
track.set_allow_unused_references(True)
track.set_track_reference_position_derivatives(True)
track.set_apply_tracked_states_to_guess(True)
track.set_initial_time(0.0)
track.set_final_time(0.47008941)
study = track.initialize()
problem = study.updProblem()


# Goals
# =====

# Symmetry
# --------
# This goal allows us to simulate only one step with left-right symmetry
# that we can then double to create a full gait cycle.
symmetryGoal = osim.MocoPeriodicityGoal("symmetryGoal")
problem.addGoal(symmetryGoal)
model = modelProcessor.process()
model.initSystem()

# Symmetric coordinate values (except for pelvis_tx) and speeds. Here, we 
# constrain final coordinate values of one leg to match the initial value of the 
# other leg. Or, in the case of the pelvis_tx value, we constrain the final 
# value to be the same as the initial value.
for i in range(model.getNumStateVariables()):
    currentStateName = model.getStateVariableNames().getitem(i)
    if currentStateName.startswith("/jointset"):
        if currentStateName.__contains__("_r"):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r"_r", "_l", currentStateName))
            symmetryGoal.addStatePair(pair)
        if currentStateName.__contains__("_l"):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r"_l", "_r", currentStateName))
            symmetryGoal.addStatePair(pair)
        if not (currentStateName.__contains__("_r") 
                or currentStateName.__contains__("_l") 
                or currentStateName.__contains__("pelvis_tx/value") 
                or currentStateName.__contains__("/activation")):
            symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(currentStateName))

# Symmetric muscle activations. Here, we constrain final muscle activation 
# values of one leg to match the initial activation values of the other leg.
for i in range(model.getNumStateVariables()):
    currentStateName = model.getStateVariableNames().getitem(i)
    if currentStateName.endswith("/activation"):
        if currentStateName.__contains__("_r"):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r"_r", "_l", currentStateName))
            symmetryGoal.addStatePair(pair)
        if currentStateName.__contains__("_l"):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r"_l", "_r", currentStateName))
            symmetryGoal.addStatePair(pair)

# The lumbar coordinate actuator control is symmetric.
symmetryGoal.addControlPair(osim.MocoPeriodicityGoalPair("/lumbarAct"))

# Get a reference to the MocoControlGoal that is added to every MocoTrack
# problem by default and change the weight
effort = osim.MocoControlGoal.safeDownCast(problem.updGoal("control_effort"))
effort.setWeight(controlEffortWeight)

# Optionally, add a contact tracking goal.
if GRFTrackingWeight != 0:
    # Track the right and left vertical and fore-aft ground reaction forces.
    contactTracking = osim.MocoContactTrackingGoal("contact", GRFTrackingWeight)
    contactTracking.setExternalLoadsFile("referenceGRF.xml")
    forceNamesRightFoot = osim.StdVectorString()
    forceNamesRightFoot.append("contactHeel_r")
    forceNamesRightFoot.append("contactFront_r")
    contactTracking.addContactGroup(forceNamesRightFoot, "Right_GRF")
    forceNamesLeftFoot = osim.StdVectorString()
    forceNamesLeftFoot.append("contactHeel_l")
    forceNamesLeftFoot.append("contactFront_l")
    contactTracking.addContactGroup(forceNamesLeftFoot, "Left_GRF")
    contactTracking.setProjection("plane")
    contactTracking.setProjectionVector(osim.Vec3(0, 0, 1))
    problem.addGoal(contactTracking)


# Bounds
# ======
problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value", [-20*math.pi/180, -10*math.pi/180])
problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", [0, 1])
problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value", [0.75, 1.25])
problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value", [-10*math.pi/180, 60*math.pi/180])
problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value", [-10*math.pi/180, 60*math.pi/180])
problem.setStateInfo("/jointset/knee_l/knee_angle_l/value", [-50*math.pi/180, 0])
problem.setStateInfo("/jointset/knee_r/knee_angle_r/value", [-50*math.pi/180, 0])
problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value", [-15*math.pi/180, 25*math.pi/180])
problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value", [-15*math.pi/180, 25*math.pi/180])
problem.setStateInfo("/jointset/lumbar/lumbar/value", [0, 20*math.pi/180])


# Solve the problem
# =================
gaitTrackingSolution = study.solve()

# Create a full stride from the periodic single step solution.
# For details, view the Doxygen documentation for createPeriodicTrajectory().
fullStride = osim.createPeriodicTrajectory(gaitTrackingSolution)
fullStride.write("gaitTracking_solution_fullStride.sto")

# Uncomment next line to visualize the result
# study.visualize(fullStride)


# Extract ground reaction forces
# ==============================
contact_r = osim.StdVectorString()
contact_l = osim.StdVectorString()
contact_r.append("contactHeel_r")
contact_r.append("contactFront_r")
contact_l.append("contactHeel_l")
contact_l.append("contactFront_l")

externalForcesTableFlat = osim.createExternalLoadsTableForGait(model, 
                                                               fullStride,contact_r,contact_l)
osim.STOFileAdapter.write(externalForcesTableFlat, 
                          "gaitTracking_solutionGRF_fullStride.sto")


# Uncomment next line to terminate after solving only the tracking problem
# sys.exit()



#------------------------------------------------------------------------
# Set up a gait prediction problem where the goal is to minimize effort
# (squared controls) divided by distance traveled while enforcing symmetry of
# the walking cycle and a prescribed average gait speed through endpoint
# constraints. The solution of the coordinate tracking problem is
# used as an initial guess for the prediction.


# Define the optimal control problem
# ==================================
study = osim.MocoStudy()
study.setName("gaitPrediction")

problem = study.updProblem()
modelProcessor = osim.ModelProcessor("2D_gait.osim")
problem.setModelProcessor(modelProcessor)


# Goals
# =====

# Symmetry (to permit simulating only one step)
symmetryGoal = osim.MocoPeriodicityGoal("symmetryGoal")
problem.addGoal(symmetryGoal)
model = modelProcessor.process()
model.initSystem()

# Symmetric coordinate values (except for pelvis_tx) and speeds
for i in range(model.getNumStateVariables()):
    currentStateName = model.getStateVariableNames().getitem(i)
    if currentStateName.startswith("/jointset"):
        if currentStateName.__contains__("_r"):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r"_r", "_l", currentStateName))
            symmetryGoal.addStatePair(pair)
        if currentStateName.__contains__("_l"):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r"_l", "_r", currentStateName))
            symmetryGoal.addStatePair(pair)
        if not (currentStateName.__contains__("_r") 
                or currentStateName.__contains__("_l") 
                or currentStateName.__contains__("pelvis_tx/value") 
                or currentStateName.__contains__("/activation")):
            symmetryGoal.addStatePair(osim.MocoPeriodicityGoalPair(currentStateName))

# Symmetric muscle activations
for i in range(model.getNumStateVariables()):
    currentStateName = model.getStateVariableNames().getitem(i)
    if currentStateName.endswith("/activation"):
        if currentStateName.__contains__("_r"):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r"_r", "_l", currentStateName))
            symmetryGoal.addStatePair(pair)
        if currentStateName.__contains__("_l"):
            pair = osim.MocoPeriodicityGoalPair(currentStateName, 
                                                re.sub(r"_l", "_r", currentStateName))
            symmetryGoal.addStatePair(pair)

# Symmetric coordinate actuator controls
symmetryGoal.addControlPair(osim.MocoPeriodicityGoalPair("/lumbarAct"))


# Prescribed average gait speed
speedGoal = osim.MocoAverageSpeedGoal("speed")
problem.addGoal(speedGoal)
speedGoal.set_desired_average_speed(1.2)

# Effort over distance
effortGoal = osim.MocoControlGoal("effort", 10)
problem.addGoal(effortGoal)
effortGoal.setExponent(3)
effortGoal.setDivideByDisplacement(True)


# Bounds
# ======
problem.setTimeBounds(0, [0.4, 0.6])
problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value", [-20*math.pi/180, -10*math.pi/180])
problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", [0, 1])
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
solver.set_num_mesh_intervals(50)
solver.set_verbosity(2)
solver.set_optim_solver("ipopt")
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-4)
solver.set_optim_max_iterations(1000)
solver.setGuess(gaitTrackingSolution) # Use tracking solution as initial guess

# Solve problem
# =============
gaitPredictionSolution = study.solve()

# Create a full stride from the periodic single step solution.
# For details, view the Doxygen documentation for createPeriodicTrajectory().
fullStride = osim.createPeriodicTrajectory(gaitPredictionSolution)
fullStride.write("gaitPrediction_solution_fullStride.sto")


# Visualize the result.
study.visualize(fullStride)

# Extract ground reaction forces
# ==============================
contact_r = osim.StdVectorString()
contact_l = osim.StdVectorString()
contact_r.append("contactHeel_r")
contact_r.append("contactFront_r")
contact_l.append("contactHeel_l")
contact_l.append("contactFront_l")

# Create a conventional ground reaction forces file by summing the contact
# forces of contact spheres on each foot.
# For details, view the Doxygen documentation for
# createExternalLoadsTableForGait().
externalForcesTableFlat = osim.createExternalLoadsTableForGait(model, 
                                                               fullStride, contact_r, contact_l)
osim.STOFileAdapter.write(externalForcesTableFlat, 
                          "gaitPrediction_solutionGRF_fullStride.sto")