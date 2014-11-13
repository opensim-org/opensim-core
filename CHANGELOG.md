When possible, we provide the GitHub issues or pull requests that
are related to the items below. If there is no issue or pull
request related to the change, then we may provide a commit hash.

v4.0
====

Converting from v3.2 to v4.0
-----------------------------
- The Actuator class has been renamed to ScalarActuator (and Actuator_ has been renamed to Actuator) (PR #126).
  If you have subclassed from Actuator, you must now subclass from ScalarActuator. 
- Methods like `Actuator::getForce` are renamed to use "Actuator" instead (e.g., `Actuator::getActuator`) (PR #209).
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

Other changes
-------------
- There is now a formal CMake mechanism for using OpenSim in your own C++ project. See cmake/SampleCMakeLists.txt.
