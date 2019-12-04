- 2019-12-02: Fixed multiple examples by updating "cost" to "goal", and other 
              such changes. Fixed exampleMocoTrack by ignoring tendon 
              compliance.

- 2019-12-02: Add a MATLAB MocoTrajectory plotting utility, 
              osimMocoTrajectoryReport.m. This has the same functionality as 
              report.py in Moco's python package.

- 2019-11-28: Brian Umberger contributed a Matlab version of example2DWalking,
              which contains a 2-D prediction of walking.

- 2019-11-26: Consistently refer to MocoTrajectories as "trajectory" instead of
              "iterate".

- 2019-11-22: Introduce TabOpUseAbsoluteStateNames to convert column labels
              from IK solutions pre-4.0 states files to use new-style column
              labels.

- 2019-11-09: Improve the reliability of building the Ipopt dependency by using
              more stable servers for downloading Metis and MUMPS.

- 2019-11-18: Updates to report.py linewidth and legend formatting. 

- 2019-11-18: Utility createPeriodicTrajectory() now properly handles 
              antisymmetric coordinate position, speed, and actuator 
              variables.    

- 2019-10-30: ModelFactory::replaceMusclesWithPathActuators() now adds the
              PathActuators to the Model's ForceSet, and the connectee names
              for PathPoints are now valid.

- 2019-10-30: Solvers print the date and time before and after solving a 
              problem.

- 2019-10-27: DeGrooteFregly2016Muscle::replaceMuscles() now carries over the
              appliesForce property. This affects 
              ModOpReplaceMusclesWithDeGrooteFregly2016 as well.

- 2019-10-19: configureMoco.m adds Moco's Matlab Utilities directory
              to the Matlab path, and removes any detected OpenSense beta
              installations from Matlab.

- 2019-10-18: MocoInverse has separate properties for constraint and convergence
              tolerances.

- 2019-10-16: Fix a bug in ModOpscaleMaxIsometricForce, where the scale factor
              was not used properly.

- 2019-10-12: Add GetMocoVersion(), GetMocoVersionAndDate() to provide the 
              git commit hash and commit date. The opensim-moco command-line
              tool now has a --version flag to print the Moco version.
              
- 2019-10-04: report.py can plot normalized tendon force states. Users can 
              provide a MocoStudy file instead of a Model file, and users can
              specify the name of the report output file.
              
- 2019-09-29: Remove INDYGO and GlobalStaticOptimization from Moco.

- 2019-09-28: exampleSlidingMassAdvanced.cpp and exampleMocoCustomEffortGoal
              show how to create a custom goal class.

- 2019-09-23: MocoControlGoal has properties "exponent" and
              "divide_by_displacement". 

- 2019-09-05: MocoAverageSpeedGoal was added.

- 2019-08-29: Users now interface with num_mesh_intervals instead of
              num_mesh_points when setting times to sample.

- 2019-08-20: Periodicity of the states and/or controls can now be enforced with
              a negated MocoPeriodicityGoalPair added via addNegatedStatePair or
              addNegatedControlPair respectively.

- 2019-07-18: Periodicity of the states and/or controls can be enforced with
              MocoPeriodicityGoals that impose equality of the initial and final
              state/control values.

- 2019-07-18: Three new methods for MocoTrajectory are now available to compute
              missing states trajectory data given existing data. For example,
              you may now use a guess generated for explicit dynamics in implicit
              dynamics by calling generateAccelerationsFromSpeeds() on the guess
              before passing it to the solver.

- 2019-07-15: MocoGoals can be enforced either as objective terms or as endpoint
              constraints. Applying a MocoGoal as an endpoint constraint is
              supported by MocoCasADiSolver (not MocoTropterSolver).

- 2019-07-09: Cost terms can now depend on initial states/controls, not just
              final states/controls. Endpoint and integral costs are combined: a
              single cost can depend on both an integral and initial/final
              states/controls. This change is necessary to support costs like
              metabolic cost of transport, which depends on both the integral of
              metabolic rate and the difference between final and initial states
              (for distance traveled).

