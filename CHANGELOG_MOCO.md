Moco Change Log
===============

1.2.1
-----
- 2023-03-21: Fixed a bug where failing `MocoProblem`s with path constraints returned
              a zero time vector.

- 2023-03-08: Added `MocoTrajectory::trimToIndices`.

- 2023-02-25: Added `getSphereForce`, `getHalfSpaceForce`, and associated `Output`s 
              `sphere_force` and `half_space_force` to `SmoothSphereHalfSpaceForce`.

- 2023-01-24: Added convenience methods `MocoGoal::setEndpointConstraintBounds` and
              `MocoGoal::getEndpointConstraintBounds`.

- 2023-01-03: Add center of pressure calculations to 
              `MocoUtilities::createExternalLoadsTableForGait`.

- 2022-07-25: Added property `normalize_tracking_error` to `MocoContactTrackingGoal` 
              to normalize the 3D contact tracking error based on the contact 
              tracking data.

1.2.0
-----
- 2023-01-19: Added MocoOutputExtremumGoal.

- 2022-09-07: Added MocoContactImpulseTrackingGoal.

- 2022-06-03: Fixed bug that was breaking marker tracking problems when
              using MocoTrack::setMarkersReferenceFromTRC().

- 2022-04-15: Added MocoInitialOutputGoal, MocoFinalOutputGoal,
              MocoOutputPeriodicityGoal, MocoOutputTrackingGoal, and
              MocoOutputConstraint.

- 2022-03-25: Fixed a bug where calculations from MocoStudy::analyze() were
              incorrect because DeGrooteFregly2016Muscle auxiliary variables
              were not included in the SimTK::State. Added convenience 
              accessors to MocoTrajectory and fixed a bug where
              MocoTrajectory::generateAccelerationsFromValues() and
              MocoTrajectory::generateAccelerationsFromSpeeds() were
              overwriting non-acceleration derivatives in the trajectory.

- 2022-02-10: Added option to MocoCasADiSolver to enforce MocoPathConstraints 
              at mesh interval midpoints when using Hermite-Simpson collocation
              (via the property 'enforce_path_constraint_midpoints').

- 2022-02-01: Updated MocoOutputGoal to support SimTK::Vec3 and SimTK::SpatialVec 
              types and custom integrand exponents.
              
- 2022-01-25: Added option for automatic variable scaling based on trajectory 
              variable bounds to MocoCasADiSolver via the property
              'scale_variables_using_bounds'.

- 2021-11-18: Implemented fix for DeGrooteFregly2016Muscle so that optimal fiber 
              lengths and tendon slack lengths are scaled when using the ScaleTool.
  
- 2021-11-17: Added Matlab and Python support for MocoAngularVelocityTrackingGoal.
              Fixed Matlab and Python issues with 
              MocoOrientationTrackingGoal::setRotationReference(); this method 
              is now overloaded to accept tables of type TimeSeriesTableQuaternion.

1.1.0
-----
- 2021-07-13: Added MocoStepTimeAsymmetryGoal, MocoStepLengthAsymmetryGoal, and
              example2DWalkingStepAsymmetry (Matlab).

- 2021-07-09: Added MocoScaleFactor support for MocoStateTrackingGoal, 
              MocoMarkerTrackingGoal, and MocoContactTrackingGoal.
  
- 2021-06-29: Added Matlab version of example2DWalkingMetabolics (via Brian 
              Umberger).
              
- 2021-06-28: Added exampleIMUTracking (Matlab and Python) for TGCS 2021.

- 2021-06-28: Added exampleEMGTracking (Matlab and Python) for TGCS 2021.
  
- 2021-06-28: Added support for optimized scale factors including the component
              MocoScaleFactor, and interface for MocoGoals to add scale factors
              to a MocoProblem (e.g., MocoControlTrackingGoal::addScaleFactor).

- 2021-02-24: Updated MocoAccelerationTrackingGoal to add support for tracking
              acceleration signals from inertial measurement units.
  
1.0.0
-----
- 2021-01-11: An Exception is now thrown if the model includes joints whose
              generalized speeds do not match the derivative of the generalized
              coordinates (i.e., BallJoint, FreeJoint, EllipsoidJoint, and
              ScapulothoracicJoint).
  
- 2020-12-10: Fixed bugs where ModOpReplaceMusclesWithDeGrooteFregly2016 didn't 
              include the muscle PathWrapSet and muscle wrapping wasn't 
              thread-safe (preventing parallelization with MocoCasADiSolver).

- 2020-11-17: Add support for solving a MocoStudy to opensim-cmd.

- 2020-08-19: DeGrooteFregly2016Muscle now reports the equilibrium residual 
              as a unitless quantity (normalized by maximum isometric force).

- 2020-08-17: The analyze() utility function has been split into a new 
              analyze() function that takes a StatesTrajectory and an
              analyzeMocoTrajectory() function that takes a MocoTrajectory.

- 2020-08-11: TableProcessor::processRadians() is renamed to 
              processAndConvertToRadians().
              
- 2020-08-10: The MocoStudy property write_solution was split into separate
              write_solution (bool) and results_directory properties. MocoStudy 
              no longer write the solution to file by default.

- 2020-08-07: MocoStudy::setModelCopy() and MocoPhase::setModelCopy() are 
              renamed as setModelAsCopy()

- 2020-07-30: MocoTrack::solve() no longer takes a boolean argument for 
              visualizing the solution. Use MocoTrack::solveAndVisualize()
              instead.

- 2020-07-28: TableProcessor no longer automatically coverts tables from 
              degrees to radians. Use TableProcessor::processRadians() or
              TabOpConvertDegreesToRadians instead.

- 2020-07-15: Add exampleHangingMuscle.cpp and exampleHangingMuscle.py that 
              show how to use MuscleAnalysis and metabolic probes with a Moco
              solution.
              
- 2020-07-15: Fix an issue with the DeGrooteFregly2016Muscle that caused
              MuscleAnalysis to produce NaNs for certain quantities.
              
- 2020-06-22: Moco can be built without CasADi, and the functions
              MocoCasADiSolver::isAvailable() and 
              MocoTropterSolver::isAvailable() are added.

- 2020-06-20: Use a more recent commit of CasADi from github.com/casadi (instead
              of using our own fork).

- 2020-06-08: Moco uses OpenSim's new message logging system.

- 2020-06-01: Introduce MocoGoal::getStageDependency() to improve efficiency of 
              goals.
              
- 2020-06-01: MocoSolvers no longer directly set the model's control cache. 
              Instead, Moco adds a DiscreteController to the model and the 
              solvers modify the DiscreteController's control signals.
              This change allows Moco to support synergies in the future.

- 2020-05-24: Expose MocoAccelerationTrackingGoal in Matlab and Python.
              
- 2020-05-16: Moved ActivationCoordinateActuator from opensim-moco to
              opensim-core.

- 2020-05-11: Update names of mesh files in 2D_gait.osim.

- 2020-05-11: Add outputs to DeGrooteFregly2016Muscle for passive elastic and 
              damping forces.

- 2020-05-11: Fix an updated function name in exampleSquatToStand.m.

- 2020-04-16: Added exampleKinematicConstraints.py to visualize how Moco handles
              kinematic constraints for a simple planar point mass.


0.4.0 
-----
- 2020-04-03: Added missing MATLAB and Python commands for getting/setting 
              derivative and multiplier variables to the bindings.

- 2020-03-29: (backwards-incompatible) Updated opensim-core to a version that 
              includes SmoothSphereHalfSpaceForce. Therefore, 
              SmoothSphereHalfSpaceForce was removed from Moco.
              The interface of this class has been updated
              to use OpenSim's ContactSphere and ContactHalfSpace classes. 
              See the following link for how to update your models:
              https://github.com/opensim-org/opensim-moco/commit/ccd39f82acba5f23ebc3423eb40d2197d6d41d8b#diff-eb627df1f2a265a84a30b7c6b4f92999
              
- 2020-03-27: Fix bug in MocoPeriodicityGoal that prevented convergence when
              constraining control variables.

- 2020-03-25: Fixed an error that occurs when using ModOpRemoveMuscles.

- 2020-03-15: exampleMocoInverse now employs electromyography data with a 
              MocoControlTrackingGoal.
              
- 2020-03-15: MocoControlTrackingGoal provides more flexibility when associating
              control signals with reference data.

- 2020-03-10: The default parameters for createPeriodicTrajectory() are fixed
              (pelvis_rotation and lumbar_rotation are now properly negated).

- 2020-02-06: example2DWalking now includes a MocoContactTrackingGoal 
              (thanks to Brian Umberger and Antoine Falisse).

- 2020-01-29: DeGrooteFregly2016Muscle's passive force multiplier now has a
              stiffness parameter. To support this change, the curve was 
              tweaked slightly. This may have a minor effect on solutions
              relying on passive fiber forces. The maximum difference between
              the new and old passive force-length curves is 0.07 (unitless).

- 2020-01-28: Added a projection setting to MocoFrameDistanceConstraint.

- 2020-01-27: MocoContactTrackingGoal now handles scenarios where contact 
              spheres are distributed across multiple bodies.


0.3.0 
-----
- 2020-01-15: MocoContactTrackingGoal allows minimizing the error between
              compliant contact forces and experimentally measured contact 
              forces.

- 2020-01-10: MocoSolution now provides a breakdown of terms in the cost 
              (only when using MocoCasADiSolver).

- 2020-01-08: exampleSitToStand renamed to exampleSquatToStand. 

- 2019-12-24: Added a bisection root solver.

- 2019-12-17: Fix a bug in report.py related to the now nonexistent 
              STOFileAdapter.read.
              

0.2.0
-----
- 2019-12-12: Added MocoFrameDistanceConstraint.

- 2019-12-11: Add MocoOutputGoal, allowing any scalar model output to be used as
              as a goal.

- 2019-12-11: ActivationCoordinateActuator now provides default activation 
              bounds equal to its control bounds.
              
- 2019-12-11: Add ModOpFiberDampingDFG to set the fiber damping for all
              DeGrooteFregly2016Muscles in a model.

- 2019-12-10: Add an example for MocoInverse in MATLAB, Python, and C++.

- 2019-12-10: Update the Simbody dependency to version 3.7.

- 2019-12-09: MocoTrajectory::isCompatible() gives more details when provided 
              with an incompatible problem.

- 2019-12-08: Include support for OpenSim's C3DFileAdapter.

- 2019-12-04: Updated the version of OpenSim used by Moco. As a result, we
              removed the utility readTableFromFile(). Use the TimeSeriesTable 
              constructor instead, which accepts a filename.
              
- 2019-12-04: Removed support for approximating GeometryPaths with a generic 
              function.

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

- 2019-11-25: Update SmoothSphereHalfSpaceForce to visualize contact forces in
              the Simbody visualizer (not the OpenSim GUI).

- 2019-11-22: Introduce TabOpUseAbsoluteStateNames to convert column labels  
              from IK solutions pre-4.0 states files to use new-style column 
              labels. 

- 2019-11-20: Added MocoAngularVelocityTrackingGoal and 
              MocoAccelerationTrackingGoal in anticipation of supporting 
              applications using IMU data in the future.
              
- 2019-11-18: Updates to report.py linewidth and legend formatting. 

- 2019-11-18: Exporting controls to TimeSeriesTable via 
              MocoTrajectory::exportToControlsTable().

- 2019-11-18: Utility createPeriodicTrajectory() now properly handles 
              antisymmetric coordinate position, speed, and actuator 
              variables.    
              
- 2019-11-18: Add a "Getting started" page to the User Guide.

- 2019-11-09: Improve the reliability of building the Ipopt dependency by using
              more stable servers for downloading Metis and MUMPS.

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

- 2019-09-28: exampleSlidingMassAdvanced.cpp and exampleMocoCustomEffortGoal
              show how to create a custom goal class.
              
              
0.1.0-preprint
--------------
- 2019-11-05: Updated the documentation to contain a list of all examples.

- 2019-10-16: Fix a bug in ModOpScaleMaxIsometricForce, where the scale factor
              was not used properly.

- 2019-10-12: Add GetMocoVersion(), GetMocoVersionAndDate() to provide the 
              git commit hash and commit date. The opensim-moco command-line
              tool now has a --version flag to print the Moco version.
              
- 2019-10-04: MocoControlGoal weights can be specified via regular expression
              patterns.

- 2019-10-04: report.py can plot normalized tendon force states. Users can 
              provide a MocoStudy file instead of a Model file, and users can
              specify the name of the report output file.
              
- 2019-09-29: Remove INDYGO and GlobalStaticOptimization from Moco.

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

