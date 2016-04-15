This document lists the changes to `opensim-core` that are
introduced with each new version, starting with version 4.0. When possible, we provide the
GitHub issues or pull requests that
are related to the items below. If there is no issue or pull
request related to the change, then we may provide the commit.

This is not a comprehensive list of changes but rather a hand-curated collection of the more notable ones. For a comprehensive history, see the [OpenSim Core GitHub repo](https://github.com/opensim/opensim-core).

**Note**: This document is currently under construction.

v4.0 (in development)
=====================

Converting from v3.x to v4.0
-----------------------------
- The Actuator class has been renamed to ScalarActuator (and Actuator_ has been renamed to Actuator) (PR #126).
  If you have subclassed from Actuator, you must now subclass from ScalarActuator.
- Methods like `Actuator::getForce` are renamed to use "Actuator" instead (e.g., `Actuator::getActuator`) (PR #209).
- Markers are now ModelComponents (PR #188). Code is included for conversion on serialization/de-serialization.
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

Bug Fixes
---------
- Fixed a typo in one of the method names for AbstractTool verifyUniqueComulnLabels -> verifyUniqueColumnLabels (PR #130)
- Fixed bug where Body VisibleObject was not serialized when writing a model to XML (PR #139)
- Fixed memory leaks in AssemblySolver and using Simtk::XML (PR #176)
- Fixed model mass scaling. When 'preserve mass distribution' is unchecked (GUI) the input mass was previously not respected and the resulting scaled model mass does not equal the input mass. The modelscaler() now alters the body masses and inertias to match the input mass. (PR #230)
- Fixed a bug in the equilibrium solution of Millard and Thelen muscles, where the initial activation and fiber-length values (for solving for equilibrium) were always coming from the default values. This was unnecessary, because unless specified otherwise, the state automatically contains the default values. This fixes an issue where inital states activations from a file were not respected by the Forward Tool and instead, the initial activations would revert to the model defaults. (PR #272)
- Fixed a bug where MuscleAnalysis was producing empty moment arm files. We now avoid creating empty Moment and MomentArm storage files when _computeMoments is False. (PR #324)

New Classes
-----------
- Added a BodyActuator component, which applies a spatial force on a specified Point of a Body (PR #126)
- Created Frame, PhysicalFrame, OffsetFrame, PhysicalOffsetFrame, Station and Marker ModelComponents (PR #188, PR #325, PR #339). Marker did not previously comply with the Model Component interface.
- A Body is a PhysicalFrame
- Connections to Bodies upgraded to PhysicalFrames and locations on these frames are now represented by PhysicalOffsetFrame (PR #370)
- Joints were refactored so that the base Joint manages the parent and child frame connections, including the definition of local PhysicalOffsetFrames to handle offsets defined as separate location and orientation properties. (PR #589)  
- The WeldConstraint and BushingForces (BushingForce, CoupledBushingForce, FunctionBasedBushingForce, and ExpressionBasedBushingForce) were similarly unified (like Joints) to handle the two Frames that these classes require to operate. A LinkTwoFrames intermediate class was introduced to house the common operations. Convenience constructors for WeldConstraint and BushingFrames were affected and now require the name of the Component as the first argument. (PR #649)
- The new StatesTrajectory class allows users to load an exact representation of previously-computed states from a file. (PR #730)

Removed Classes
--------------------------------
The following classes are no longer supported in OpenSim and are removed in OpenSim 4.0.
- Muscle class `ContDerivMuscle_Depredated`.

Python
------
- Improved error handling. Now, OpenSim's error messages show up as exceptions
in Python.

Other Changes
-------------
- There is now a formal CMake mechanism for using OpenSim in your own C++ project. See cmake/SampleCMakeLists.txt. (PR #187)
- Substantial cleanup of the internal CMake scripts.
- Lepton was upgraded to the latest version (PR #349)
- Made Object::print a const member function (PR #191)
- Improved the testOptimization/OptimizationExample to reduce the runtime (PR #416)
- InverseKinematics tool outputs marker error .sto file if report error flag is true.
- Marker location file output name in IK changed to reflect trial name for batch processing. 

Documentation
--------------
- Improved Doxygen layout and fixed several bugs and warnings (various)
- All mentions of SimTK/Simbody classes in OpenSim's Doxygen now provide links directly to SimTK/Simbody's doxygen.
- Added a detailed README.md wtith build instructions, as well as guides to contributing and developing (CONTRIBUTING.md).
- Included GIFs in Doxygen for several commonly used Joint types

STILL NEED TO ADD:
- Additional changes to Model Component Interface, Iterator (any PRs labeled "New MCI")
- PR #364
- PR #370
- PR #378
