## Part 0: Load the Moco libraries and pre-configured Models.
# These models are provided for you (i.e., they are not part of Moco).
import opensim as osim
import exampleSquatToStand_helpers as helpers
import mocoPlotTrajectory as plot
import os
import numpy as np
torqueDrivenModel = helpers.getTorqueDrivenModel()
muscleDrivenModel = helpers.getMuscleDrivenModel()

## Part 1: Torque-driven Predictive Problem
# Part 1a: Create a new MocoStudy.
study = osim.MocoStudy()

# Part 1b: Initialize the problem and set the model.
problem = study.updProblem()
problem.setModel(torqueDrivenModel)

# Part 1c: Set bounds on the problem.
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

# Part 1d: Add a MocoControlCost to the problem.
problem.addGoal(osim.MocoControlGoal('myeffort'))

# Part 1e: Configure the solver.
solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(25)
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-4)

if not os.path.isfile('predictSolution.sto'):
    # Part 1f: Solve! Write the solution to file, and visualize.
    predictSolution = study.solve()
    predictSolution.write('predictSolution.sto')
    study.visualize(predictSolution)


## Part 2: Torque-driven Tracking Problem
# Part 2a: Construct a tracking reference TimeSeriesTable using filtered 
# data from the previous solution. Use a TableProcessor, which accepts a 
# base table and allows appending operations to modify the table.
tableProcessor = osim.TableProcessor('predictSolution.sto')
tableProcessor.append(osim.TabOpLowPassFilter(6))

# Part 2b: Add a MocoStateTrackingCost to the problem using the states
# from the predictive problem (via the TableProcessor we just created). 
# Enable the setAllowUnusedReferences() setting to ignore the controls in
# the predictive solution.
tracking = osim.MocoStateTrackingGoal()
tracking.setName('mytracking')
tracking.setReference(tableProcessor)
tracking.setAllowUnusedReferences(True)
problem.addGoal(tracking)

# Part 2c: Reduce the control cost weight so it now acts as a regularization 
# term.
problem.updGoal('myeffort').setWeight(0.001)

# Part 2d: Set the initial guess using the predictive problem solution.
# Tighten convergence tolerance to ensure smooth controls.
solver.setGuessFile('predictSolution.sto')
solver.set_optim_convergence_tolerance(1e-6)

if not os.path.isfile('trackingSolution.sto'):
    # Part 2e: Solve! Write the solution to file, and visualize.
    trackingSolution = study.solve()
    trackingSolution.write('trackingSolution.sto')
    study.visualize(trackingSolution)


## Part 3: Compare Predictive and Tracking Solutions
# This is a convenience function provided for you. See mocoPlotTrajectory.m
plot.mocoPlotTrajectory('predictSolution.sto', 'trackingSolution.sto', 
    'predict', 'track')

## Part 4: Muscle-driven Inverse Problem
# Create a MocoInverse tool instance.
inverse = osim.MocoInverse()

# Part 4a: Provide the model via a ModelProcessor. Similar to the TableProcessor,
# you can add operators to modify the base model.
modelProcessor = osim.ModelProcessor(muscleDrivenModel)
modelProcessor.append(osim.ModOpAddReserves(2))
inverse.setModel(modelProcessor)

# Part 4b: Set the reference kinematics using the same TableProcessor we used
# in the tracking problem.
inverse.setKinematics(tableProcessor)

# Part 4c: Set the time range, mesh interval, and convergence tolerance.
inverse.set_initial_time(0)
inverse.set_final_time(1)
inverse.set_mesh_interval(0.05)
inverse.set_convergence_tolerance(1e-4)
inverse.set_constraint_tolerance(1e-4)

# Allow extra (unused) columns in the kinematics and minimize activations.
inverse.set_kinematics_allow_extra_columns(True)
inverse.set_minimize_sum_squared_activations(True)

# Append additional outputs path for quantities that are calculated
# post-hoc using the inverse problem solution.
inverse.append_output_paths('.*normalized_fiber_length')
inverse.append_output_paths('.*passive_force_multiplier')

# Part 4d: Solve! Write the MocoSolution to file.
inverseSolution = inverse.solve()
solution = inverseSolution.getMocoSolution()
solution.write('inverseSolution.sto')

# Part 4e: Get the outputs we calculated from the inverse solution.
inverseOutputs = inverseSolution.getOutputs()
sto = osim.STOFileAdapter()
sto.write(inverseOutputs, 'muscleOutputs.sto')

## Part 5: Muscle-driven Inverse Problem with Passive Assistance
# Part 5a: Create a new muscle-driven model, now adding a SpringGeneralizedForce 
# about the knee coordinate.
device = osim.SpringGeneralizedForce('knee_angle_r')
device.setStiffness(50)
device.setRestLength(0)
device.setViscosity(0)
muscleDrivenModel.addForce(device)

# Part 5b: Create a ModelProcessor similar to the previous one, using the same
# reserve actuator strength so we can compare muscle activity accurately.
modelProcessor = osim.ModelProcessor(muscleDrivenModel)
modelProcessor.append(osim.ModOpAddReserves(2))
inverse.setModel(modelProcessor)

# Part 5c: Solve! Write solution.
inverseDeviceSolution = inverse.solve()
deviceSolution = inverseDeviceSolution.getMocoSolution()
deviceSolution.write('inverseDeviceSolution.sto')

## Part 6: Compare unassisted and assisted Inverse Problems.
print('Cost without device: ', solution.getObjective())
print('Cost with device: ', deviceSolution.getObjective())

# This is a convenience function provided for you. See below for the
# implementation.
helpers.compareInverseSolutions(inverseSolution, inverseDeviceSolution)
