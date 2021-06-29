## Part 0: Load the OpenSim and Moco libraries.
from opensim import *
from exampleEMGTracking_helpers import *
import os
import numpy as np

## Part 1: Muscle redundancy problem: effort minimization.
# Solve the muscle redundancy problem while minimizing muscle excitations
# squared using the MocoInverse tool.

# Part 1a: Load a 19 degree-of-freedom model with 18 lower-limb,
# sagittal-plane muscles and a torque-actuated torso. This includes a set of
# ground reaction forces applied to the model via ExternalLoads, which is
# necessary for the muscle redundancy problem. See the function definition
# at the bottom of this file to see how the model is loaded and constructed.
model = getWalkingModel()

# Part 1b: Create the MocoInverse tool and set the Model.


# Part 1c: Create a TableProcessor using the coordinates file from inverse
# kinematics.


# Part 1d: Set the kinematics reference for MocoInverse using the
# TableProcessor we just created.


# Part 1e: Provide the solver settings: initial and final time, the mesh
# interval, and the constraint and convergence tolerances.


if not os.path.isfile('effortSolution.sto'):
    # Part 1f: Solve the problem!


##  Part 2: Plot the muscle redundancy problem solution.
# Load the experimental electromyography data and compare
# the effort minimization solution against this data. We will also use
# it later for the EMG-tracking problem. Each column in emg.sto is
# normalized so the maximum value for each signal is 1.0.
emgReference = TimeSeriesTable('emg.sto')
compareSolutionToEMG(emgReference, 'effortSolution.sto')

## Part 3: Muscle redundancy problem: EMG-tracking.
# Modify the existing problem we created with the MocoInverse tool to solve
# a new problem where we will track electromyography (EMG) data.

# Part 3a: Call initialize() to get access to the MocoStudy contained within
# the MocoInverse instance. This will allow us to make additional
# modifications to the problem not provided by MocoInverse.


# Part 3b: Create a MocoControlTrackingGoal, set its weight, and provide
# the EMG data as the tracking reference. We also need to specify the
# reference labels for the four muscles whose EMG we will track.


# Part 3c: The EMG signals in the tracking are all normalized to have
# a maximum value of 1, but the magnitudes of the excitations from the
# effort minimization solution suggest that these signals should be
# rescaled. Use addScaleFactor() to add a MocoParameter to the problem that
# will scale the reference data for the muscles in the tracking cost.


# Part 3d: Add the tracking goal to the problem.


# Part 3e: Update the MocoCasADiSolver with the updated MocoProblem using
# resetProblem().


# Part 3f: Tell MocoCasADiSolver that the MocoParameters we added to the
# problem via addScaleFactor() above do not require initSystem() calls on
# the model. This provides a large speed-up.


if not os.path.isfile('trackingSolution.sto'):
    # Part 3g: Solve the problem!


# Part 3h: Get the values of the optimized scale factors.


## Part 4: Plot the EMG-tracking muscle redundancy problem solution.
# Part 4a: Print the scale factor values to the command window.
print('\nOptimized scale factor values:')
print('------------------------------')
print('gastrocnemius = ' + str(gastroc_factor))
print('tibialis anterior = ' + str(tibant_factor))
print('biceps femoris short head = ' + str(bifem_factor))
print('gluteus = ' + str(gluteus_factor))

# Part 4b: Re-scale the reference data using the optimized scale factors.
gastroc = emgReference.updDependentColumn('gastrocnemius')
tibant = emgReference.updDependentColumn('tibialis_anterior')
bifem = emgReference.updDependentColumn('biceps_femoris')
gluteus = emgReference.updDependentColumn('gluteus')
for t in np.arange(emgReference.getNumRows()):
    t = int(t) # Convert to Python built-in int type for indexing
    gastroc[t] = gastroc_factor * gastroc[t]
    tibant[t] = tibant_factor * tibant[t]
    bifem[t] = bifem_factor * bifem[t]
    gluteus[t] = gluteus_factor * gluteus[t]

# Part 4c: Generate the plots. Compare results to the effort minimization
# solution.
compareSolutionToEMG(emgReference, 'effortSolution.sto',
    'trackingSolution.sto')
