INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors ${OpenSim_SOURCE_DIR}/Vendors/xerces-c-src2_4_0/src)


LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(convertControls rdConvertControls.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(convertStorageToControls rdConvertStorageToControls.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(simplifyControls rdSimplifyControls.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(perturbStorage perturbStorage.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(storageAdd storageAdd.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(storageIntegrate storageIntegrate.cpp)

LINK_LIBRARIES( debug xerces-c_2D optimized        xerces-c_2 rdTools rdSimulation)
ADD_EXECUTABLE(storageSubtract storageSubtract.cpp)

LINK_LIBRARIES( debug xerces-c_2D  optimized xerces-c_2 rdTools)
ADD_EXECUTABLE(storageFilter storageFilter.cpp)

INSTALL_TARGETS(/bin/ convertControls)
INSTALL_TARGETS(/bin/ convertStorageToControls)
INSTALL_TARGETS(/bin/ simplifyControls)
INSTALL_TARGETS(/bin/ perturbStorage)
INSTALL_TARGETS(/bin/ storageAdd)
INSTALL_TARGETS(/bin/ storageIntegrate)
INSTALL_TARGETS(/bin/ storageSubtract)
INSTALL_TARGETS(/bin/ storageFilter)
