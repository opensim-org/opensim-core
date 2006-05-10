INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors ${OpenSim_SOURCE_DIR}/Vendors/xerces-c-src2_4_0/src)


LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(rdConvertControls rdConvertControls.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(rdConvertStorageToControls rdConvertStorageToControls.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(rdSimplifyControls rdSimplifyControls.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(suPerturbStorage perturbStorage.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(suStorageAdd storageAdd.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(suStorageIntegrate storageIntegrate.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(suStorageSubtract storageSubtract.cpp)

INSTALL_TARGETS(/bin/ rdConvertControls)
INSTALL_TARGETS(/bin/ rdConvertStorageToControls)
INSTALL_TARGETS(/bin/ rdSimplifyControls)
INSTALL_TARGETS(/bin/ suPerturbStorage)
INSTALL_TARGETS(/bin/ suStorageAdd)
INSTALL_TARGETS(/bin/ suStorageIntegrate)
INSTALL_TARGETS(/bin/ suStorageSubtract)
