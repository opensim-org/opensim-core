INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors ${OpenSim_SOURCE_DIR}/Vendors/xerces-c-src2_4_0/src)


LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(convertControls rdConvertControls.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(convertStorageToControls rdConvertStorageToControls.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(simplifyControls rdSimplifyControls.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(serturbStorage perturbStorage.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(suStorageAdd storageAdd.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(storageIntegrate storageIntegrate.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(storageSubtract storageSubtract.cpp)

LINK_LIBRARIES( debug xerces-c_2D  optimized xerces-c_2 rdTools)
ADD_EXECUTABLE(storageFilter storageFilter.cpp)

INSTALL_TARGETS(/bin/ rdConvertControls)
INSTALL_TARGETS(/bin/ rdConvertStorageToControls)
INSTALL_TARGETS(/bin/ rdSimplifyControls)
INSTALL_TARGETS(/bin/ suPerturbStorage)
INSTALL_TARGETS(/bin/ suStorageAdd)
INSTALL_TARGETS(/bin/ suStorageIntegrate)
INSTALL_TARGETS(/bin/ suStorageSubtract)
INSTALL_TARGETS(/bin/ storageFilter)
