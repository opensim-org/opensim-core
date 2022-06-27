# -*- coding: utf-8 -*-
"""
MOCO: WALKING 2D EXAMPLE - STEP ASYMMETRY

@author: Prasanna Sritharan, June 2022


This is a Python implementation of an example optimal control problem
originally written by Russell T. Johnson for Matlab 
(see example2DWalkingStepAsymmetry.m).

Simulate asymmetric gait using MocoStepTimeAsymmetryGoal and 
MocoStepLengthAsymmetryGoal. This is an extension of the example2DWalking 
MATLAB example (see example2DWalking.m for details about model and data used).
"""


from math import pi
import opensim as osim
import os
import numpy as np



"""
stepTimeAsymmetry():
    
Set up a predictive optimization problem where the goal is to minimize an 
effort cost (cubed controls) and hit a target step time asymmetry. Unlike 
example2DWalking, this problem requires simulating a full gait cycle. 
Additionally, endpoint constraints enforce periodicity of the coordinate values 
(except for pelvis tx) and speeds, coordinate actuator controls, and muscle 
activations.
 
Step time is defined as the time between consecutive foot strikes. Step Time 
Asymmetry (STA) is a ratio and is calculated as follows:
 - Right Step Time (RST) = Time from left foot-strike to right foot-strike
 - Left Step Time (LST)  = Time from right foot-strike to left foot-strike
 - STA = (RST - LST) / (RST + LST)
 
The step time goal works by "counting" the number of nodes that each foot is in 
contact with the ground (with respect to a specified contact force threshold). 
Since, in walking, there are double support phases where both feet are on the 
ground, the goal also detects which foot is in front and assigns the step time 
to the leading foot. Altogether, it estimates the time between consecutive 
heel strikes in order to infer the left and right step times.
 
The contact elements for each foot must specified via 'setLeftContactGroup()'
and 'setRightContactGroup()'. The force element and force threshold used to 
determine when a foot is in contact is set via 'setContactForceDirection()' and 
'setContactForceThreshold()'.
 
Users must provide the target asymmetry value via 'setTargetAsymmetry()'.
Asymmetry values ranges from -1.0 to 1.0. For example, 0.20 is 20positive
step time asymmetry with greater right step times than left step times. A
symmetric step times solution can be achieved by setting this property to zero.
This goal can be used only in 'cost' mode, where the error between the target
asymmetry and model asymmetry is squared. To make this goal suitable for
gradient-based optimization, step time values are assigned via smoothing
functions which can be controlled via 'setAsymmetrySmoothing()' and
'setContactDetectionSmoothing()'.
 
Since this goal doesn't directly compute the step time asymmetry from heel 
strikes, users should confirm that the step time asymmetry from the solution 
matches closely to the target. To do this, we provide the helper function 
computeStepAsymmetryValues() below.
"""
def stepTimeAsymmetry():


    # **********************************
    # DEFINE THE OPTIMAL CONTROL PROBLEM
    
    # Create a MocoStudy
    study = osim.MocoStudy()
    study.setName("step_time_asymmetry")
    
    # Get the model
    modelProcessor = osim.ModelProcessor("2D_gait.osim")
    modelProcessor.append(osim.ModOpTendonComplianceDynamicsModeDGF("implicit"))
    
    # Get the MocoProblem from the MocoStudy and set the model on it
    problem = study.updProblem()
    problem.setModelProcessor(modelProcessor)
    
    
    
    # **********************************
    # SET GOALS
    
    # Periodicity:
    # All states are periodic except pelvis_tx value, lumbar actuator control
    # is periodic.
    periodicityGoal = osim.MocoPeriodicityGoal("periodicity")
    model = modelProcessor.process()
    model.initSystem()
    state_names = [model.getStateVariableNames().getitem(sn) for sn in range(model.getNumStateVariables())]
    for sn in state_names:
        if "pelvis_tx/value" not in sn:
            periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn))
    periodicityGoal.addControlPair(osim.MocoPeriodicityGoalPair("/lumbarAct"))
    problem.addGoal(periodicityGoal)

    # Average gait speed
    speedGoal = osim.MocoAverageSpeedGoal("avg_speed")
    speedGoal.set_desired_average_speed(1.0)
    problem.addGoal(speedGoal)
    
    # Effort
    effortGoal = osim.MocoControlGoal("effort", 10.0)
    effortGoal.setExponent(3)
    effortGoal.setDivideByDisplacement(True)
    problem.addGoal(effortGoal)
    
    # Step time asymmetry:
    # Create the goal, and configure it
    stepTimeAsymmetryGoal = osim.MocoStepTimeAsymmetryGoal("step_time_asymmetry")
    # Value for smoothing term used to compute when foot contact is made
    # (default = 0.25). Users may need to adjust this based on convergence and 
    # matching the target asymmetry.
    stepTimeAsymmetryGoal.setContactDetectionSmoothing(0.4)
    # Contact threshold based on vertical GRF (default = 25 N)
    stepTimeAsymmetryGoal.setContactForceThreshold(25.0)
    # Value for smoothing term used to compute asymmetry (default = 10). Users
    # may need to adjust this based on convergence and matching the target
    # asymmetry.
    stepTimeAsymmetryGoal.setAsymmetrySmoothing(3.0)
    # Target step length asymmetry. Positive values mean greater right step
    # length than left.
    stepTimeAsymmetryGoal.setTargetAsymmetry(0.10)
    # Set goal weight
    stepTimeAsymmetryGoal.setWeight(5.0)
    # Need to define the names of the left and right heel spheres: this is
    # used to detect which foot is in front during double support phase.
    contact_r = ["contactHeel_r", "contactFront_r"]
    contact_l = ["contactHeel_l", "contactFront_l"]
    stepTimeAsymmetryGoal.setRightContactGroup(contact_r, "contactHeel_r")
    stepTimeAsymmetryGoal.setLeftContactGroup(contact_l, "contactHeel_l")
    # Add the goal to the problem
    problem.addGoal(stepTimeAsymmetryGoal)
    
    
    
    # **********************************
    # SET BOUNDS    

    # Set time bounds
    problem.setTimeBounds(0.0, 0.94)
    
    # Coordinate bounds as dict
    coord_bounds = {}
    coord_bounds["/jointset/groundPelvis/pelvis_tilt/value"] = [-20*pi/180, 20*pi/180]
    coord_bounds["/jointset/groundPelvis/pelvis_tx/value"] = [0, 2]
    coord_bounds["/jointset/groundPelvis/pelvis_ty/value"] = [0.75, 1.25]
    coord_bounds["/jointset/hip_r/hip_flexion_r/value"] = [-10*pi/180, 60*pi/180]
    coord_bounds["/jointset/hip_l/hip_flexion_l/value"] = [-10*pi/180, 60*pi/180]
    coord_bounds["/jointset/knee_r/knee_angle_r/value"] = [-50*pi/180, 0]
    coord_bounds["/jointset/knee_l/knee_angle_l/value"] = [-50*pi/180, 0]
    coord_bounds["/jointset/ankle_r/ankle_angle_r/value"] = [-15*pi/180, 25*pi/180]
    coord_bounds["/jointset/ankle_l/ankle_angle_l/value"] = [-15*pi/180, 25*pi/180]
    coord_bounds["/jointset/lumbar/lumbar/value"] = [0, 20*pi/180]
    
    # Set coordinate bounds
    for bnd in coord_bounds:
        problem.setStateInfo(bnd, coord_bounds[bnd])
    
    
    
    # **********************************
    # SOLVE

    # Configure the solver    
    solver = study.initCasADiSolver()
    solver.set_num_mesh_intervals(100)
    solver.set_verbosity(2)
    solver.set_optim_convergence_tolerance(1e-4)
    solver.set_optim_constraint_tolerance(1e-4)
    solver.set_optim_max_iterations(2000)

    # Set the initial guess to be the symmetric two-steps tracking solution
    # from example2DWalking.py. Run this first, or proceed without a guess.
    guess_file = "walk_2D_two_steps_tracking_solution.sto"
    if os.path.isfile(guess_file):
        solver.setGuessFile(guess_file)
        
    # Print the Study to file
    study.printToXML("example2DWalkingStepTimeAsymmetry.omoco")
        
    
    # Solve
    solution = study.solve()
    solution.write("walk_2D_step_time_asym_solution.sto")
    
    
    # Write predicted GRF to file
    contact_forces_table = osim.createExternalLoadsTableForGait(model, solution, contact_r, contact_l)
    osim.STOFileAdapter().write(contact_forces_table, "walk_2D_step_time_asym_ground_forces.sto")
    
    # Calculate step time asymmetry
    step_time_asym, _ = computeStepAsymmetry(model, 25, "walk_2D_step_time_asym_solution.sto", "walk_2D_step_time_asym_ground_forces.sto")
    print("\nStep time asymmetry: %f\n" % step_time_asym)


    return study, solution



"""
stepLengthAsymmetry():

This goal works by limiting the distance between feet, or "foot frames", 
throughout the gait cycle. The goal calculates the distance between the left 
foot and right foot, then limits the distance between feet to not pass beyond 
minimum (negative) or maximum (positive) bounds. There are two limits used: 
one that limits the distance between feet when the right foot is in front, and 
one that limits the distance between feet when the left foot is in front.

Step Length Asymmetry (SLA) is a ratio and is calculated as follows:
The Right Step Length (RSL) is the distance between feet at right foot strike
The Left Step Length (LSL) is the distance between feet at left foot strike
Step Length Asymmetry = (RSL - LSL)/ (RSL + LSL) 

Users must provide the target asymmetry value via 'setTargetAsymmetry()'.
Asymmetry values ranges from -1.0 to 1.0. For example, 0.20 is 20positive
step length asymmetry with greater right step length than left step length. A
symmetric step length solution can be achieved by setting this property to zero.
This goal can be used only in 'cost' mode, where the error between the target
asymmetry and model asymmetry is squared. To make this goal suitable for
gradient-based optimization, step length values are assigned via a smoothing
function which can be controlled via 'setAsymmetrySmoothing()'.

Users must also prescribed the stride length via 'setStrideLength()'. The goal 
then calculates the minimum and maximum bounds on the distance between right 
and left foot. Users must ensure that this stride length is met via problem
bounds or other goals; the value provided to MocoStepLengthAsymmetryGoal is 
only used to compute the model's asymmetry in the cost function.

Because this goal doesn't directly compute the step length asymmetry from
heel strike data, users should confirm that the step length asymmetry
from the solution matches closely to their target. To do this, we
provide the helper function computeStepAsymmetryValues() below. Users may
also want to confirm that the stride length from the optimization
matches with setStrideLength(), or set additional constraints for stride length
within the optimization. Additionally, in some cases users may want to set 
target asymmetries above or below the desired value, in the event there is 
some offset.
"""
def stepLengthAsymmetry():


    # **********************************
    # DEFINE THE OPTIMAL CONTROL PROBLEM
    
    # Create a MocoStudy
    study = osim.MocoStudy()
    study.setName("step_length_asymmetry")
    
    # Get the model
    modelProcessor = osim.ModelProcessor("2D_gait.osim")
    modelProcessor.append(osim.ModOpTendonComplianceDynamicsModeDGF("implicit"))
    
    # Get the MocoProblem from the MocoStudy and set the model on it
    problem = study.updProblem()
    problem.setModelProcessor(modelProcessor)
    
    
    
    # **********************************
    # SET GOALS
    
    # Periodicity:
    # All states are periodic except pelvis_tx value, lumbar actuator control
    # is periodic.
    periodicityGoal = osim.MocoPeriodicityGoal("periodicity")
    model = modelProcessor.process()
    model.initSystem()
    state_names = [model.getStateVariableNames().getitem(sn) for sn in range(model.getNumStateVariables())]
    for sn in state_names:
        if "pelvis_tx/value" not in sn:
            periodicityGoal.addStatePair(osim.MocoPeriodicityGoalPair(sn))
    periodicityGoal.addControlPair(osim.MocoPeriodicityGoalPair("/lumbarAct"))
    problem.addGoal(periodicityGoal)

    # Average gait speed
    speedGoal = osim.MocoAverageSpeedGoal("avg_speed")
    speedGoal.set_desired_average_speed(1.0)
    problem.addGoal(speedGoal)
    
    # Effort
    effortGoal = osim.MocoControlGoal("effort", 10.0)
    effortGoal.setExponent(3)
    effortGoal.setDivideByDisplacement(True)
    problem.addGoal(effortGoal)
    
    # Step length asymmetry:
    # Create the goal, and configure it
    stepLengthAsymmetryGoal = osim.MocoStepLengthAsymmetryGoal()
    # Set body names for left and right foot
    stepLengthAsymmetryGoal.setRightFootFrame('/bodyset/calcn_r')
    stepLengthAsymmetryGoal.setLeftFootFrame('/bodyset/calcn_l')
    # Provide the stride length for the simulation (m)
    stepLengthAsymmetryGoal.setStrideLength(0.904)
    # Value for smoothing term used to compute asymmetry (default = 5). Users
    # may need to adjust this based on convergence and matching the target
    # asymmetry.
    stepLengthAsymmetryGoal.setAsymmetrySmoothing(5)
    # Target step length asymmetry. Positive values mean greater right step
    # length than left.
    stepLengthAsymmetryGoal.setTargetAsymmetry(-0.10)
    # Set goal weight
    stepLengthAsymmetryGoal.setWeight(5)
    # Add the goal to the problem
    problem.addGoal(stepLengthAsymmetryGoal)
    
    
    
    # **********************************
    # SET BOUNDS    

    # Set time bounds
    problem.setTimeBounds(0.0, 0.94)
    
    # Coordinate bounds as dict
    coord_bounds = {}
    coord_bounds["/jointset/groundPelvis/pelvis_tilt/value"] = [-20*pi/180, 20*pi/180]
    coord_bounds["/jointset/groundPelvis/pelvis_tx/value"] = [0, 2]
    coord_bounds["/jointset/groundPelvis/pelvis_ty/value"] = [0.75, 1.25]
    coord_bounds["/jointset/hip_r/hip_flexion_r/value"] = [-10*pi/180, 60*pi/180]
    coord_bounds["/jointset/hip_l/hip_flexion_l/value"] = [-10*pi/180, 60*pi/180]
    coord_bounds["/jointset/knee_r/knee_angle_r/value"] = [-50*pi/180, 0]
    coord_bounds["/jointset/knee_l/knee_angle_l/value"] = [-50*pi/180, 0]
    coord_bounds["/jointset/ankle_r/ankle_angle_r/value"] = [-15*pi/180, 25*pi/180]
    coord_bounds["/jointset/ankle_l/ankle_angle_l/value"] = [-15*pi/180, 25*pi/180]
    coord_bounds["/jointset/lumbar/lumbar/value"] = [0, 20*pi/180]
    
    # Set coordinate bounds
    for bnd in coord_bounds:
        problem.setStateInfo(bnd, coord_bounds[bnd])
    
    
    
    # **********************************
    # SOLVE

    # Configure the solver    
    solver = study.initCasADiSolver()
    solver.set_num_mesh_intervals(100)
    solver.set_verbosity(2)
    solver.set_optim_convergence_tolerance(1e-4)
    solver.set_optim_constraint_tolerance(1e-4)
    solver.set_optim_max_iterations(2000)

    # Set the initial guess to be the symmetric two-steps tracking solution
    # from example2DWalking.py. Run this first, or proceed without a guess.
    guess_file = "walk_2D_two_steps_tracking_solution.sto"
    if os.path.isfile(guess_file):
        solver.setGuessFile(guess_file)
        
    # Print the Study to file
    study.printToXML("example2DWalkingStepLengthAsymmetry.omoco")
        
    
    # Solve
    solution = study.solve()
    solution.write("walk_2D_step_length_asym_solution.sto")
    
    # Write predicted GRF to file
    contact_r = ["contactHeel_r", "contactFront_r"]
    contact_l = ["contactHeel_l", "contactFront_l"]
    contact_forces_table = osim.createExternalLoadsTableForGait(model, solution, contact_r, contact_l)
    osim.STOFileAdapter().write(contact_forces_table, "walk_2D_step_length_asym_ground_forces.sto")
    
    # Calculate step time asymmetry
    _, step_length_asym = computeStepAsymmetry(model, 25, "walk_2D_step_length_asym_solution.sto", "walk_2D_step_length_asym_ground_forces.sto")
    print("\nStep length asymmetry: %f\n" % step_length_asym)
    
   
    return study, solution



"""
computeStepAsymmetry():
    
Calculate the values of the step length and step time asymmetry from the
results of the simulation.
"""
def computeStepAsymmetry(model_file, threshold, solution_file, grf_file):
    
    # Load model
    model = osim.Model(model_file)
    
    # Load predicted GRF
    grf = np.genfromtxt(grf_file, skip_header=5, delimiter="\t")
    
    # GRF time vector and vertical components
    tvec = grf[:, 0]


    # **********************************
    # STEP TIME ASYMMETRY
    
    # Find index of heel strike on each leg
    hs_idxR = findHeelStrikeIndex(grf[:, 2], threshold)
    hs_idxL = findHeelStrikeIndex(grf[:, 8], threshold)
   
    # Compute step time on each leg
    hs_timeR = tvec[hs_idxR]
    hs_timeL = tvec[hs_idxL]
    if hs_timeR < hs_timeL:
        step_timeL = hs_timeL - hs_timeR
        step_timeR = tvec[-1] - step_timeL
    else:
        step_timeR = hs_timeR - hs_timeL
        step_timeL = tvec[-1] - step_timeR  
    
    # Calculate step time asymmetry (%)
    step_time_asym = 100 * (step_timeR - step_timeL) / (step_timeR + step_timeL)
    
    
    # **********************************
    # STEP LENGTH ASYMMETRY
    
    # Get the states for each limb at the instant of heel strike on that limb
    states_traj = osim.StatesTrajectory().createFromStatesTable(model, osim.TimeSeriesTable(solution_file), False, True, True)
    statesR = states_traj.get(hs_idxR)
    statesL = states_traj.get(hs_idxL)
    
    # Calculate the step length
    step_lengthR = calculateStepLength(model, statesR)
    step_lengthL = calculateStepLength(model, statesL)
    
    # Calculate step length asymmetry (%)
    step_length_asym = 100 * (step_lengthR - step_lengthL) / (step_lengthR + step_lengthL)
    
    
    return step_time_asym, step_length_asym



"""
findHeelStrikeIndex():
    
Find heel strike index by determining foot contact on-off instances. If no
heel strike is found, then assume first index is heel strike. This 
implementation differs from that of Russell T. Johnson's Matlab version, but
follows the same prinicples.
"""
def findHeelStrikeIndex(grfy, threshold):
    
    # Find windows representing ground contact
    is_contact = (grfy > threshold).astype(int)

    # Calculate transition points, i.e. heel strike (1) and foot off (-1)
    contact_diff = np.diff(np.insert(is_contact, 0, 1))
    
    # Extract heel strike and foot off indices. If no heel strike found, 
    # assume first index is heel strike.
    idxs = np.where(contact_diff == 1)[0]
    if idxs.size == 0:
        idx = 0
    else:
        idx = idxs[0]
        
    return int(idx)
    
    

"""
calculateStepLength():
    
Find step length by configuring the model at heel strike, then compute distance
between contact spheres along the fore-aft coordinate.
"""    
def calculateStepLength(model, state):
    
    # Configure the model at heel strike
    model.initSystem()
    model.realizePosition(state)
    
    # Get the heel contact spheres
    contact_r = model.getContactGeometrySet().get("heel_r")
    contact_l = model.getContactGeometrySet().get("heel_l")
    
    # Find the positions of the contact spheres in the global frame
    pos_r = contact_r.getFrame().getPositionInGround(state)
    pos_l = contact_l.getFrame().getPositionInGround(state)
    
    # Step length is the difference between global position of the left and
    # right along the fore-aft coordinate (x)
    step_length = abs(pos_r.get(0) - pos_l.get(0))
    
    return step_length
    



# %% STEP TIME ASYMMETRY

# Solve and visualise
step_time_study, step_time_solution = stepTimeAsymmetry()
step_time_study.visualize(step_time_solution)



# %% STEP LENGTH ASYMMETRY

# Solve and visualise
step_length_study, step_length_solution = stepLengthAsymmetry()
step_length_study.visualize(step_length_solution)
   
