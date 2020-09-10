This document lists the changes to `opensim-core` that are
introduced with each new version, starting with version 4.0. When possible, we provide the
GitHub issues or pull requests that
are related to the items below. If there is no issue or pull
request related to the change, then we may provide the commit.

This is not a comprehensive list of changes but rather a hand-curated collection of the more notable ones. For a comprehensive history, see the [OpenSim Core GitHub repo](https://github.com/opensim-org/opensim-core).


v4.2
====
- Add the ActivationCoordinateActuator component, which is a CoordinateActuator with simple activation dynamics (PR #2699).
- Easily convert Matlab matrices and Python NumPy arrays to and from OpenSim Vectors and Matrices. See Matlab example matrixConversions.m and Python example numpy_conversions.py.
- Users have more control over which messages are logged. Messages are now logged to opensim.log instead of out.log and err.log. Users can control logging levels via `Logger::setLevel()`.
- Fix a segfault that occurs when using OpenSim's Python Package with Anaconda's Python on a Mac.
- Expose PropertyHelper class to python bindings to allow editing of objects using the properties interface (useful for editing objects defined in plugins) in python (consistent with Java/Matlab).
- Whitespace is trimmed when reading table metadata for STO, MOT, and CSV files.
- Introduce utilities for creating SimTK::Vectors, linear interpolation, updating table column labels from v3.3 to v4.0 syntax, solving for a function's root using bisection (OpenSim/Common/CommonUtilities.h) ([PR #2808](https://github.com/opensim-org/opensim-core/pull/2808)).
- Introduce utilities for querying, filtering, and resampling TimeSeriesTables (OpenSim/Common/TableUtilities.h) ([PR #2808](https://github.com/opensim-org/opensim-core/pull/2808)).
- StatesTrajectories can now be created from a TimeSeriesTable of states.
- Minor performance improvements (5-10 %) for controller-heavy models (PR #2806)
- `Controller::isEnabled` will now only return whether the particular controller is enabled
  - Previously, it would return `false` if its parent `Model`'s `Model::getAllControllersEnabled` returned `false`
  - The previous behavior would mean that `Controller::setEnabled(true); return Controller::isEnabled();` could return `false`
- The new Matlab examplePointMass.m shows how to build and simulate a point-mass model.
- Fix OpenSense calibration algorithm to handle models facing an arbitrary direction. The calibration algorithm now aligns one axis of the provided Orientation Sensor data with the x-axis of the base segment (e.g. pelvis) of the model in default pose.
- For PrescribedController, the controls_file column labels can now be absolute paths to actuators (previously, the column labels were required to be actuator names).
- Fixed a critical bug in Induced Accelerations Analysis which prevents analysis to run when external forces are present ([PR #2847](https://github.com/opensim-org/opensim-core/pull/2808)).
- Removed `Path` abstract base class (PR #2844)
  - Unused by OpenSim and related projects
- Improved the performance of `ComponentPath` (PR #2844)
  - This improves the performance of component-heavy models by ~5-10 %
  - The behavior and interface of `ComponentPath` should remain the same
- The new Matlab CustomStaticOptimization.m guides the user to build their own custom static optimization code. 


v4.1
====
- Added `OrientationsReference` as the frame orientation analog to the location of experimental markers. Enables experimentally measured orientations from wearable sensors (e.g. from IMUs) to be tracked by reference frames in the model. A correspondence between the experimental (IMU frame) orientation column label and that of the virtual frame on the `Model` is expected. The `InverseKinematicsSolver` was extended to simultaneously track the `OrientationsReference` if provided. (PR #2412)
- Removed the undocumented `bool dumpName` argument from `Object::dump()` and made the method `const` so it can be safely called on `const` objects. (PR #2412)
- `MarkersReference` convenience constructors were updated to take a const reference to a `MarkerWeightSet` as its second argument. If a `Set` is not empty, then only the markers listed are used as reference signals. That means `InverseKinematicsTool` no longer tracks all experimental markers even those not in the `MarkerWeightSet`. One can quickly track all experimental markers (that have a corresponding model marker) by simply providing an empty `Set`, in which case all markers are assigned the default weight (typically 1.0).
- Model files from very old versions (pre 1.8.1) are not supported, an exception is thrown rather than fail quietly (issue #2395).
- Initializing a Component from an existing Component with correct socket connectees yields invalid paths (issue #2418).
- Reading DataTables from files has been simplified. Reading one table from a file typically uses the Table constructor except when the data-source/file contains multiple tables. (In these cases e.g. C3D files, use C3DFileAdapter.read method, then use functions in C3DFileAdapter to get the individual TimeSeriesTable(s)). Writing tables to files has not changed.
- Exposed convertMillimeters2Meters() in osimC3D.m. This function converts COP and moment data from mm to m and now must be invoked prior to writing force data to file. Previously, this was automatically performed during writing forces to file. 
- Methods that operate on SimTK::Vec<n> are now available through Java/Matlab and python bindings to add/subtract/divide/multiply vec<n> contents with a scalar (PR #2558)
- The new Stopwatch class allows C++ API users to easily measure the runtime of their code.
- If finalizeConnections() method was not called on a model after making changes and before printing, an exception is thrown to avoid creating corrupt model files quietly (PR #2529)
- Updated the docopt.cpp dependency so that OpenSim can be compiled with Visual C++ from Visual Studio 2019.
- Added `Blankevoort1991Ligament` force component which represents ligament fibers as non-linear path springs. The force-strain curve has a quadratic toe region at low strains and a linear stiffness region at high strains. (PR #2632)  
- Updated Simbody to 3.7 to fix an issue with the simbody-visualizer on macOS 10.15 Catalina.
- On Mac and Linux, we include a shell script opensim-install-command-line.sh to make OpenSim's command-line tools easily accessible.
- Added the compliant SmoothSphereHalfSpaceForce component, for use with direct collocation and Moco.
 

Converting from v4.0 to v4.1
----------------------------
- The `OpenSim::Array` constructor is now marked explicit, which prevents
  accidental implicit conversion to `Array`. If you relied on this implicit
  conversion, you will need to update your code to use the constructor
  explicitly.

Bug Fixes
---------
- Fixed bug in osimTable2Struct.m for renaming unlabelled markers (PR #2491)
- Fixed bug that resulted in an exception when reading C3D files without forces. Now, if the C3D doesn't contain markers or forces, an empty table will be returned (PR #2421) 
- Fix bug that resulted in activations and forces reported for Actuators that are disabled during StaticOptimization (issue #2438) Disabled actuators are now ignored in StaticOptimization.
- OpenSim no longer supports model file formats predating version 1.8.1 (PR #2498)
- FunctionBasedBushingForce now applies damping if specified (it was incorrectly ignored in 4.0) issue #2512
- TRCFileAdapter.write() uses the number of columns and rows in the supplied dataTable to set the "NumMarkers" and "NumRows" Metadata in the output file. Users won't have to set this metadata string manually.  #2510

Documentation
-------------


Other Changes
-------------
- Performance of reading large data files has been significantly improved. A 50MB .sto file would take 10-11 min to read now takes 2-3 seconds. (PR #2399)
- Added Matlab example script of plotting the Force-length properties of muscles in a models; creating an Actuator file from a model; 
building and simulating a simple arm model;  using OutputReporters to record and write marker location and coordinate values to file.
- Added Python example that demonstrates how to run an optimization using the cma package and how to avoid an expensive call to `initSystem()` within the objective function. (PR #2604)
- OpenSim 4.1 ships with Python3 bindings as default. It is still possible to create bindings for Python2 if desired by setting CMake variable OPENSIM_PYTHON_VERSION to 2
- For CMake, the option OPENSIM_COPY_DEPENDENCIES option is now an advanced option, and a warning is provided if this option is off but wrapping is turned on.

v4.0
====

Converting from v3.x to v4.0
-----------------------------
- A significant difference between v3.3 and 4.0 is the naming of dependencies. Unique names were not enforced in 3.3, which led to undefined behavior. In 4.0, Component pathnames must be unique. That is a Component must be unique with respect to its peers. A Model named *model* cannot have multiple subcomponents with the name *toes* either as bodies or joints, because the pathname */model/toes* will not uniquely identify the Component. However, multiple *toes* bodies can be used as long as they are not subcomponents of the same Component. For example, a *device* Component with a *toes* Body will have no issues since this *toes* Body has a unique pathname, */model/device/toes*, which is unambiguous. One could also create a multi-legged model, where each leg is identical, with *hip* and *knee* joints and *upper* and *lower* bodies, but each being unique because each `Leg` Component that contains the leg subcomponents, is uniquely named like */model/leg1* and */model/leg4/* and thus all of their subcomponents are unique, e.g.: */model/leg1/knee* vs. */model/leg4/knee*.
- Component naming is more strictly enforced and names with spaces are no longer accepted. Spaces are only allowable as separators for `Output` or `Channel` names that satisfy a list `Input`. (PR #1955)
- The Actuator class has been renamed to ScalarActuator (and `Actuator_` has been renamed to `Actuator`) (PR #126).
  If you have subclassed from Actuator, you must now subclass from ScalarActuator.
- Methods like `Actuator::getForce` are renamed to use "Actuator" instead (e.g., `Actuator::getActuator`) (PR #209).
- Markers are now ModelComponents (PR #188). Code is included for conversion on serialization/de-serialization.
- MarkerSet::addMarker() was removed (PR #1898). Please use Model::addMarker() to add markers to your model.
- `Body::getMassCenter` now returns a `Vec3` instead of taking a `Vec3` reference as an argument (commit cb0697d98).
- The following virtual methods in ModelComponent have been moved:
  - connectToModel -> extendConnectToModel
  - addToSystem -> extendAddToSystem
  - initStateFromProperties -> extendInitStateFromProperties
  - setPropertiesFromState -> extendSetPropertiesFromState

  The original methods (without `extend`) still exist, but they are now non-virtual.
  To invoke `connectToModel` on an entire Model, you still call `Model::connectToModel`.
  This change has been made to make a distinction between the user interface and
  the Component developer (extension) interface. **IMPORTANT** The calls to
  `Super::addToSystem`, etc. in the implementation of these methods must now
  also use the `extend` variants. Otherwise, you will enter into an infinite recursion.
- OpenSim now makes substantial use of C++11 features; if you compile OpenSim, your compiler
  must support C++11. Also, any C++ project in which you use OpenSim must also be compiled with C++11.
- The following components have been upgraded to use Sockets to connect to
  other components they depend on (instead of string properties):
  - ContactGeometry (ContactSphere, ContactHalfSpace, ContactMesh)
- Many of the methods in ScaleTool have now been marked const.
- We created a new unified command line interface that will replace the
  single-tool command line executables (`scale`, `ik`, `id`, `rra`, `cmc`,
  etc.).
  - `scale -S setup.xml` -> `opensim run-tool setup.xml`.
  - `scale -PS` -> `opensim print-xml scale`
  - `scale -PropertyInfo ...` -> `opensim info ...`
  - `versionUpdate ...` -> `opensim update-file ...`
- The `CoordinateSet` property in `Joint` has been replaced with a `coordinates`
  list property and enumerations have been added for accessing the Coordinates
  owned by a Joint. Code like `myPlanarJoint.getCoordinateSet()[0]` now becomes
  `myPlanarJoint.getCoordinate(PlanarJoint::Coord::RotationZ)` (PRs #1116,
  #1210, and #1222).
- The `reverse` property in Joint can no longer be set by the user; Model uses
  SimTK::MultibodyGraphMaker to determine whether joints should be reversed when
  building the multibody system. The joint's transform and coordinates maintain
  a parent->child sense even if the joint has been reversed. For backwards
  compatibility, a joint's parent and child PhysicalFrames are swapped when
  opening a Model if the `reverse` element is set to `true`.
- The `MotionType` of a `Coordinate` is now fully determined by the Joint. The
  user cannot set the `MotionType` for a `Coordinate`. There are instances such
  as in the *leg6dof9musc* and *Rajagopal2015* models, where a `Coordinate` was
  assigned an incorrect type (e.g. when a coordinate of a `CustomJoint` is not a
  measure of a Cartesian angle). In 4.0, the coordinate is correctly marked as
  `Coupled` since a function couples the coordinate value to the angular
  displacement of the patella in Cartesian space. **NOTE**, this causes issues
  (e.g.  opensim-org/opensim-gui#617, #2088) when using kinematics files
  generated in 3.3 (or earlier) where `Rotational` coordinates have been
  converted to degrees. Because OpenSim 4.0 does not recognize the coordinate's
  `MotionType` to be `Rotational` it will not convert it back to radians
  internally. For motion files generated prior to 4.0 where the file has
  `inDegrees=yes`, please use the following conversion utility:
  `updatePre40KinematicsFilesFor40MotionType()`. When loading a pre-4.0 model,
  OpenSim will warn users of any changes in `MotionType` when updating an
   existing model to OpenSim 4.0.
- `Manager::integrate(SimTK::State&)` has been removed and replaced by
  `Manager::integrate(double)`. You must also now call
  `Manager::initialize(SimTK::State&)` before integrating or pass the
  initialization state into a convenience constructor. Here is a
   before-after example (see the documentation in the `Manager` class
   for more details):
  - Before:
    - Manager manager(model);
    - manager.setInitialTime(0.0);
    - manager.setFinalTime(1.0);
    - manager.integrate(state);
  - After:
    - Manager manager(model);
    - state.setTime(0.0);
    - manager.initialize(state);
    - manager.integrate(1.0);
  - After (using a convenience constructor):
    - state.setTime(0.0);
    - Manager manager(model, state);
    - manager.integrate(1.0);
- `Manager::setIntegrator(SimTK::Integrator)` has been removed and replaced by
  `Manager::setIntegratorMethod(IntegratorMethod)` which uses an enum and can
  be used by the MATLAB/Python interface. See the method's documentation for
  examples. Integrator settings are now handled by the Manager through the
  following new functions:
  - setIntegratorAccuracy(double)
  - setIntegratorMinimumStepSize(double)
  - setIntegratorMaximumStepSize(double)
  - setIntegratorInternalStepLimit(int)
- `Muscle::equilibrate(SimTK::State&)` has been removed from the Muscle interface in order to reduce the number and variety of muscle equilibrium methods. `Actuator::computeEquilibrium(SimTK::State&)` is overridden by Muscle and invokes pure virtual `Muscle::computeInitialFiberEquilibrium(SimTK::State&)`.
- `Millard2012EquilibriumMuscle::computeFiberEquilibriumAtZeroVelocity(SimTK::State&)` and `computeInitialFiberEquilibrium(SimTK::State&)` were combined into a single method:
`Millard2012EquilibriumMuscle::computeFiberEquilibrium(SimTK::State&, bool useZeroVelocity)`
where fiber-velocity can be estimated from the state or assumed to be zero if the flag is *true*.
- `Millard2012EquilibriumMuscle::computeInitialFiberEquilibrium(SimTK::State&)` invokes `computeFiberEquilibrium()` with `useZeroVelocity = true` to maintain its previous behavior.
- `Model::replaceMarkerSet()` was removed. (PR #1938) Please use `Model::updMarkerSet()` to edit the model's MarkerSet instead.
- The argument list for `Model::scale()` was changed: the `finalMass` and
  `preserveMassDist` arguments were swapped and the `preserveMassDist` argument
  is no longer optional. The default argument for `preserveMassDist` in OpenSim
  3.3 was `false`. (PR #1994)
- A GeometryPath without PathPoints is considered invalid, since it does not
represent a physical system. You must specify PathPoints to define a valid
GeometryPath for a Muscle, Ligament, PathSpring, etc... that is added to a
Model. (PR #1948)
  - Before (no longer valid):
    ```cpp
    Model model;
    Thelen2003Muscle* muscle = new Thelen2003Muscle("muscle", ...);
    // GeometryPath throws: "A valid path requires at least two PathPoints."
    model.addForce(muscle);
    ```
  - After (now required):
    ```cpp
    Model model;
    Thelen2003Muscle* muscle = new Thelen2003Muscle("muscle", ...);
    // require at least two path points to have a valid muscle GeometryPath
    muscle->addNewPathPoint("p1", ...);
    muscle->addNewPathPoint("p2", ...);
    model.addForce(muscle);
    ```
- The JointReaction analysis interface has changed in a few ways:
  - "express_in_frame" now takes a `Frame` name. "child" and "parent" keywords are also still accepted, provided that no Frame is named "child" or "parent"
  - If the number of elements in "apply_on_bodies" or "express_in_frame" is neither of length 1 or the same length as indicated by "joint_names", an exception is thrown. This was previously a warning.
- Updated wrapping properties


Composing a Component from other components
-------------------------------------------
Component now maintains a list property of *components* which it owns. You add
a (sub) Component to a *parent* Component by calling `addComponent` and passing
a heap allocated (`new Component`) to the parent which you want to take
ownership of the new subcomponent. Ownership determines how the subcomponent is serialized
(appears within the parent) and the order in which of Component interface methods (above)
are propagated to the subcomponent. Access to contained components is provided through
`getComponent<C>(path)` or `getComponentList<C>` where `C` is any Component type (default
is `Component` to get all subcomponents). These methods always traverse down into
a Component's list of components.  All subcomponents that are properties of (and thus owned by)
a parent Component are accessible this way. The Model's typed %Sets and `add####()` methods
are no longer necessary to compose a Model, since any Component can now be composed of
components. `Model` still supports `addd####()` methods and de/serialization of Sets,
but components added via `addComponent` are NOT included in the Sets but contained
in the Component's *components* property list. Details in PR#1014. **Note**, it is now
strictly required that immediate subcomponents have unique names. For example, a Model cannot contain two bodies in its `BodySet` named *tibia* or a Body and a Joint named *toes*, since it is ambiguous as to which *tibia* `Body` or *toes* `Component` is being referenced.

Bug Fixes
---------
- Fixed a typo in one of the method names for AbstractTool verifyUniqueComulnLabels -> verifyUniqueColumnLabels (PR #130)
- Fixed bug where Body VisibleObject was not serialized when writing a model to XML (PR #139)
- Fixed memory leaks in AssemblySolver and using Simtk::XML (PR #176)
- Fixed model mass scaling. When 'preserve mass distribution' is unchecked (GUI) the input mass was previously not respected and the resulting scaled model mass does not equal the input mass. The modelscaler() now alters the body masses and inertias to match the input mass. (PR #230)
- Fixed a bug in the equilibrium solution of Millard and Thelen muscles, where the initial activation and fiber-length values (for solving for equilibrium) were always coming from the default values. This was unnecessary, because unless specified otherwise, the state automatically contains the default values. This fixes an issue where initial states activations from a file were not respected by the Forward Tool and instead, the initial activations would revert to the model defaults. (PR #272)
- Fixed a bug where MuscleAnalysis was producing empty moment arm files. We now avoid creating empty Moment and MomentArm storage files when `_computeMoments` is False. (PR #324)
- Fixed bug causing the muscle equilibrium solve routine in both Thelen2003Muscle and Millard2012EquilibriumMuscle to fail to converge and erroneously return the minimum fiber length. The fix added a proper reduction in step-size when errors increase and limiting the fiber-length to its minimum. (PR #1728)
- Fixed a bug where Models with Bodies and Joints (and other component types) with the same name were loaded without error. Duplicately named Bodies were simply being ignored and only the first Body of that name in the BodySet was being used, for example, to connect a Body to its parent via its Joint, or to affix path points to its respective Body. Now, duplicate names are flagged and renamed so they are uniquely identified. (PR #1887)
- Fixed bug and speed issue with `model.setStateVariableValues()` caused by enforcing constraints after each coordinate value was being set (PR #1911). Removing the automatic enforcement of constraints makes setting all state values much faster, but also requires calling `model.assemble()` afterwards. Enforcing constraints after setting each coordinate value individually was also incorrect, since it neglected the effect of other coordinate changes have on the current coordinate. All coordinate values must be set before enforcing constraints.
- Fixed a bug that resulted in incorrect Ligament resting lengths after scaling.
  (PR #1994)

New Classes
-----------
- Added a BodyActuator component, which applies a spatial force on a specified Point of a Body (PR #126)
- Created Frame, PhysicalFrame, OffsetFrame, PhysicalOffsetFrame, Station and Marker ModelComponents (PR #188, PR #325, PR #339). Marker did not previously comply with the Model Component interface.
- A Body is a PhysicalFrame
- Connections to Bodies upgraded to PhysicalFrames and locations on these frames are now represented by PhysicalOffsetFrame (PR #370)
- Joints were refactored so that the base Joint manages the parent and child frame connections, including the definition of local PhysicalOffsetFrames to handle offsets defined as separate location and orientation properties. (PR #589)  
- The WeldConstraint and BushingForces (BushingForce, CoupledBushingForce, FunctionBasedBushingForce, and ExpressionBasedBushingForce) were similarly unified (like Joints) to handle the two Frames that these classes require to operate. A LinkTwoFrames intermediate class was introduced to house the common operations. Convenience constructors for WeldConstraint and BushingFrames were affected and now require the name of the Component as the first argument. (PR #649)
- The new StatesTrajectory class allows users to load an exact representation of previously-computed states from a file. (PR #730)
- Added Point as a new base class for all points, which include: Station, Marker, and PathPoints

- Added OutputReporter as an Analysis so that users can use the existing AnalyzeTool and ForwardTool to extract Output values of interest, without modifications to the GUI. (PR #1991)


Removed Classes
---------------
The following classes are no longer supported in OpenSim and are removed in OpenSim 4.0.
- Muscle class `ContDerivMuscle_Depredated`.

MATLAB and Python interfaces
----------------------------
- The SimbodyMatterSubsystem class--which provides operators related to the mass
matrix, Jacobians, inverse dynamics, etc.--is now accessible in MATLAB and
Python (PR #930).
- Changed wrapping of `SimTK::Array_<OpenSim::CoordinateReference>` from `ArrayCoordinateReference` to `SimTKArrayCoordinateReference` for consistency with other classes. (PR #1842)

MATLAB interface
----------------
- The configureOpenSim.m function should no longer require administrator
  privileges for most users, and gives more verbose output to assist with
  troubleshooting.
- New MATLAB examples were added: Hopper-Device and Knee-Reflex.

Python interface
----------------
- Improved error handling. Now, OpenSim's error messages show up as exceptions
in Python.
- The Python bindings can now be built for Python 3 (as well as Python 2).

Other Changes
-------------
- Support for compiling the source code with Microsoft Visual Studio 2017.
- There is now a formal CMake mechanism for using OpenSim in your own C++
  project. See cmake/SampleCMakeLists.txt. (PR #187)
- Substantial cleanup of the internal CMake scripts.
- Lepton was upgraded to the latest version (PR #349)
- Made Object::print a const member function (PR #191)
- Improved the testOptimization/OptimizationExample to reduce the runtime (PR #416)
- InverseKinematics tool outputs marker error .sto file if report error flag is true.
- Marker location file output name in IK changed to reflect trial name for batch processing.
- Created a method `ScaleTool::run()`, making it easier to run the Scale Tool
programmatically in MATLAB or python.
- Thelen2003Muscle, Millard2012EquilibriumMuscle, and
  Millard2012AccelerationMuscle now throw an exception if the force equilibrium
  calculation fails to converge (PR #1201).
- Thelen2003Muscle and Millard2012EquilibriumMuscle no longer clamp excitations (i.e. controls)
  internally. If controls are out of bounds an Exception is thrown. Also, the
  `min_control` property now defaults to the `minimum_activation`. It is the
  responsibility of the controller (or solver) to provide controls that are
  within the valid ranges defined by the Actuators and that includes the
  specific bounds of Muscle models. (PR #1548)
- The `buildinfo.txt` file, which contains the name of the compiler used to
  compile OpenSim and related information, is now named `OpenSim_buildinfo.txt`
  and may be installed in a different location.
- macOS and Linux users should no longer need to set `LD_LIBRARY_PATH` or
  `DYLD_LIBRARY_PATH` to use OpenSim libraries.
- The `scale()` method was removed from the `SimbodyEngine` class (the contents
  were moved into `Model::scale()`). (PR #1994)
- Any class derived from ModelComponent can now add its own implementation of
  `extendPreScale()`, `extendScale()`, and/or `extendPostScale()` to control how
  its properties are updated during scaling. (PR #1994)
- The source code for the "From the Ground Up: Building a Passive Dynamic
  Walker Example" was added to this repository.
- OpenSim no longer looks for the simbody-visualizer using the environment
  variable `OPENSIM_HOME`. OpenSim uses `PATH` instead.
- The Thelen2003Muscle now depend on separate components for modeling pennation,
  and activation dynamics.

Documentation
-------------
- Improved Doxygen layout and fixed several bugs and warnings (various)
- All mentions of SimTK/Simbody classes in OpenSim's Doxygen now provide links directly to SimTK/Simbody's doxygen.
- Added a detailed README.md wtith build instructions, as well as guides to contributing and developing (CONTRIBUTING.md).
- Included GIFs in Doxygen for several commonly used Joint types
