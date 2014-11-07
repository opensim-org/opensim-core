When possible, we provide the GitHub issues or pull requeststhat
are related to the items below. If there is no issue or pull
request related to the change, then we may provide a commit hash.

v4.0
====

Converting from v3.2 to v4.0
-----------------------------
- The Actuator class has been renamed to ScalarActuator (and Actuator_ has been renamed to Actuator) (PR #126).
  If you have subclassed from Actuator, you must now subclass from ScalarActuator. 
- Methods like Actuator::getForce are renamed to Actuator::getActuator (PR #209).
- Body::getMassCenter now returns a Vec3 instead of taking a Vec3 reference as an argument (commit cb0697d98).

Other changes
-------------
- There is now a formal CMake mechanism for using OpenSim in your own C++ project. See cmake/SampleCMakeLists.txt.
