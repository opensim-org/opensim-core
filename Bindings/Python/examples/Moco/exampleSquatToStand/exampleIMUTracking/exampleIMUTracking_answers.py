## Part 0: Load the Moco libraries.
import opensim as osim
import exampleIMUTracking_helpers as helpers
# Add the directory above to access mocoPlotTrajectory.py
import sys
sys.path.insert(1, '../')
import mocoPlotTrajectory as plot
import numpy as np
import os

## Part 1: Load model and add IMU frames.
# Part 1a: Load a torque-driven, 3 degree-of-freedom model with a single leg 
# and foot welded to the floor. See the function definition at the bottom of
# this file to see how the model is loaded and constructed.
model = helpers.getTorqueDrivenSquatToStandModel()

# Part 1b: Add frames to the model that will represent our IMU locations. 
# The function addIMUFrame() adds a PhysicalOffsetFrame to a body at a 
# specified location and orientation. Each frame is added at the path:
#
# /bodyset/<body_name>/<body_name>_imu_offset
#
helpers.addIMUFrame(model, 'torso', 
    osim.Vec3(0.08, 0.3, 0), osim.Vec3(0, 0.5*np.pi, 0.5*np.pi))
helpers.addIMUFrame(model, 'femur_r', 
    osim.Vec3(0, -0.2, 0.05), osim.Vec3(0, 0, 0.5*np.pi))
helpers.addIMUFrame(model, 'tibia_r', osim.Vec3(0, -0.2, 0.05), 
    osim.Vec3(0, 0, 0.5*np.pi))

# Part 1c: Add IMU components to the model using the PhysicalOffsetFrames
# we just added to the model. We'll use the helper function addModelIMUs()
# included with OpenSenseUtilities.
imuFramePaths = osim.StdVectorString()
imuFramePaths.append('/bodyset/torso/torso_imu_offset')
imuFramePaths.append('/bodyset/femur_r/femur_r_imu_offset')
imuFramePaths.append('/bodyset/tibia_r/tibia_r_imu_offset')
osim.OpenSenseUtilities().addModelIMUs(model, imuFramePaths)
model.initSystem()

## Part 2: Solve an effort minimization predictive problem
# Generate a simulation of a squat-to-stand motion that minimizes control
# effort. We'll use the results from this simulation to compute a set of 
# "synthetic" accelerometer signals later.

# Part 2a: Create a new MocoStudy.
study = osim.MocoStudy()

# Part 2b: Initialize the problem and set the model.
problem = study.updProblem()
problem.setModel(model)

# Part 2c: Set bounds on the problem.
#
# problem.setTimeBounds(initial_bounds, final_bounds)
# problem.setStateInfo(path, trajectory_bounds, inital_bounds, final_bounds)
#
# All *_bounds arguments can be set to a range, [lower upper], or to a
# single value (equal lower and upper bounds). Empty brackets, [], indicate
# using default bounds (if they exist). You may set multiple state infos at
# once using setStateInfoPattern():
#
# problem.setStateInfoPattern(pattern, trajectory_bounds, inital_bounds, ...
#       final_bounds)
#
# This function supports regular expressions in the 'pattern' argument;
# use '.*' to match any substring of the state/control path
# For example, the following will set all coordinate value state infos:
#
# problem.setStateInfoPattern('/path/to/states/.*/value', ...)

# Time bounds
problem.setTimeBounds(0, 1)

# Position bounds: the model should start in a squat and finish 
# standing up.
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value',
    [-2, 0.5], -2, 0)
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', 
    [-2, 0], -2, 0)
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', 
    [-0.5, 0.7], -0.5, 0)

# Velocity bounds: all model coordinates should start and end at rest.
problem.setStateInfoPattern('/jointset/.*/speed', [], 0, 0)

# Part 2d: Add a MocoControlGoal to the problem.
problem.addGoal(osim.MocoControlGoal('myeffort'))

# Part 2e: Configure the solver.
solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(25)
solver.set_optim_constraint_tolerance(1e-4)
solver.set_optim_convergence_tolerance(1e-4)

if not os.path.isfile('predictSolution.sto'):
    # Part 2f: Solve! Write the solution to file, and visualize.
    predictSolution = study.solve()
    predictSolution.write('predictSolution.sto')
    study.visualize(predictSolution)


##Part 3: Compute synthetic accelerometer signals
# In this step, we'll create a set of synthetic accelerometer signals that 
# replicate the output of an IMU sensor. To do this, we'll use the 
# 'accelerometer_signal' Output included with the IMU components we
# previously added to the model. 

# Part 3a: Load the prediction solution.
predictSolution = osim.MocoTrajectory('predictSolution.sto')

# Part 3b: Compute the accelerometer signals using the analyzeVec3() free
# function included with SimulationUtilities. These free functions can be
# accessed in scripting by using the 'opensimSimulation' prefix. 
outputPaths = osim.StdVectorString()
outputPaths.append('.*accelerometer_signal')
accelerometerSignals = osim.analyzeVec3(model,
    predictSolution.exportToStatesTable(),
    predictSolution.exportToControlsTable(),
    outputPaths)

# Part 3c: Update the column labels of the accelerometer signals to match
# the offset frame paths. This is necessary for the tracking goal we'll add
# to the problem in Part 4. 
accelerometerSignals.setColumnLabels(imuFramePaths)
    
# Part 3d: Plot the synthetic acceleration signals.
helpers.plotAccelerationSignals(accelerometerSignals)

## Part 4: Synthetic acceleration tracking problem 
# Part 4a: Add a MocoAccelerationTrackingGoal to the MocoProblem. Set the
# frame paths to the IMU offset frame paths and set the accelerations 
# reference to the synthetic accelerometer signals we calculated above.
# We need to subtract the gravitational acceleration vector and re-express
# the accelerations in the tracking frames so that the model-computed
# values in the tracking cost match the accelerometer signals.
tracking = osim.MocoAccelerationTrackingGoal('acceleration_tracking')
tracking.setFramePaths(imuFramePaths)
tracking.setAccelerationReference(accelerometerSignals)
tracking.setGravityOffset(True)
tracking.setExpressAccelerationsInTrackingFrames(True)
problem.addGoal(tracking)

# Part 4b: Reduce the control cost weight so that the tracking term will
# dominate.
problem.updGoal('myeffort').setWeight(0.001)

if not os.path.isfile('trackingSolution.sto'):
    # Part 4c: Solve! Write the solution to file, and visualize.
    trackingSolution = study.solve()
    trackingSolution.write('trackingSolution.sto')
    study.visualize(trackingSolution)


## Part 5: Compare tracking solution to original prediction
# Part 5a: Plot the tracking solution against the prediction. This is a
# convenience function provided for you. See mocoPlotTrajectory.m
plot.mocoPlotTrajectory('predictSolution.sto', 'trackingSolution.sto',
        'predict', 'track')
 
# Part 5b: Compare accelerations from tracking solution to the reference
# accelerations.
trackingSolution = osim.MocoTrajectory('trackingSolution.sto')
accelerometerSignalsTracking = osim.analyzeVec3(model, 
    trackingSolution.exportToStatesTable(), 
    trackingSolution.exportToControlsTable(), 
    outputPaths)
accelerometerSignalsTracking.setColumnLabels(imuFramePaths)
helpers.plotAccelerationSignals(accelerometerSignals, 
                                accelerometerSignalsTracking)
