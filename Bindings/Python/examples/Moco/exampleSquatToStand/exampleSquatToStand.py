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


# Part 1b: Initialize the problem and set the model.


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
problem.setTimeBounds( )

# Position bounds: the model should start in a squat and finish 
# standing up.
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', )
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', )
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', )

# Velocity bounds: all model coordinates should start and end at rest.
problem.setStateInfoPattern('/jointset/.*/speed', )

# Part 1d: Add a MocoControlCost to the problem.


# Part 1e: Configure the solver.


if not os.path.isfile('predictSolution.sto'):
    # Part 1f: Solve! Write the solution to file, and visualize.


## Part 2: Torque-driven Tracking Problem
# Part 2a: Construct a tracking reference TimeSeriesTable using filtered 
# data from the previous solution. Use a TableProcessor, which accepts a 
# base table and allows appending operations to modify the table.


# Part 2b: Add a MocoStateTrackingCost to the problem using the states
# from the predictive problem (via the TableProcessor we just created). 
# Enable the setAllowUnusedReferences() setting to ignore the controls in
# the predictive solution.


# Part 2c: Reduce the control cost weight so it now acts as a regularization 
# term.


# Part 2d: Set the initial guess using the predictive problem solution.
# Tighten convergence tolerance to ensure smooth controls.


if not os.path.isfile('trackingSolution.sto'):
    # Part 2e: Solve! Write the solution to file, and visualize.


## Part 3: Compare Predictive and Tracking Solutions
# This is a convenience function provided for you. See mocoPlotTrajectory.m
plot.mocoPlotTrajectory('predictSolution.sto', 'trackingSolution.sto', 
    'predict', 'track')

## Part 4: Muscle-driven Inverse Problem
# Create a MocoInverse tool instance.


# Part 4a: Provide the model via a ModelProcessor. Similar to the TableProcessor,
# you can add operators to modify the base model.


# Part 4b: Set the reference kinematics using the same TableProcessor we used
# in the tracking problem.


# Part 4c: Set the time range, mesh interval, and convergence tolerance.
inverse.set_initial_time( )
inverse.set_final_time( )
inverse.set_mesh_interval( )
inverse.set_convergence_tolerance( )
inverse.set_constraint_tolerance( )

# Allow extra (unused) columns in the kinematics and minimize activations.
inverse.set_kinematics_allow_extra_columns(True)
inverse.set_minimize_sum_squared_activations(True)

# Append additional outputs path for quantities that are calculated
# post-hoc using the inverse problem solution.
inverse.append_output_paths('.*normalized_fiber_length')
inverse.append_output_paths('.*passive_force_multiplier')

# Part 4d: Solve! Write the MocoSolution to file.


# Part 4e: Get the outputs we calculated from the inverse solution.


## Part 5: Muscle-driven Inverse Problem with Passive Assistance
# Part 5a: Create a new muscle-driven model, now adding a SpringGeneralizedForce 
# about the knee coordinate.


# Part 5b: Create a ModelProcessor similar to the previous one, using the same
# reserve actuator strength so we can compare muscle activity accurately.


# Part 5c: Solve! Write solution.


## Part 6: Compare unassisted and assisted Inverse Problems.
print('Cost without device: ', solution.getObjective())
print('Cost with device: ', deviceSolution.getObjective())

# This is a convenience function provided for you. See below for the
# implementation.
helpers.compareInverseSolutions(inverseSolution, inverseDeviceSolution)
