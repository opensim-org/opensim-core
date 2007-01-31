INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors ${XERCES_INCLUDE_DIR})

IF(NOT OPENSIM_MINIMAL_BUILD)
ADD_EXECUTABLE(convertControls rdConvertControls.cpp)
TARGET_LINK_LIBRARIES(convertControls debug xerces-c_2D optimized xerces-c_2 rdTools rdSimulation)

ADD_EXECUTABLE(convertStorageToControls rdConvertStorageToControls.cpp)
TARGET_LINK_LIBRARIES(convertStorageToControls debug xerces-c_2D optimized xerces-c_2 rdTools rdSimulation)

ADD_EXECUTABLE(simplifyControls rdSimplifyControls.cpp)
TARGET_LINK_LIBRARIES(simplifyControls debug xerces-c_2D optimized xerces-c_2 rdTools rdSimulation)

ADD_EXECUTABLE(perturbStorage perturbStorage.cpp)
TARGET_LINK_LIBRARIES(perturbStorage debug xerces-c_2D optimized xerces-c_2 rdTools rdSimulation)

ADD_EXECUTABLE(storageAdd storageAdd.cpp)
TARGET_LINK_LIBRARIES(storageAdd debug xerces-c_2D optimized xerces-c_2 rdTools rdSimulation)

ADD_EXECUTABLE(storageIntegrate storageIntegrate.cpp)
TARGET_LINK_LIBRARIES(storageIntegrate debug xerces-c_2D optimized xerces-c_2 rdTools rdSimulation)

ADD_EXECUTABLE(storageInterpolate storageInterpolate.cpp)
TARGET_LINK_LIBRARIES(storageInterpolate debug xerces-c_2D optimized xerces-c_2 rdTools rdSimulation)

ADD_EXECUTABLE(storageSubtract storageSubtract.cpp)
TARGET_LINK_LIBRARIES(storageSubtract debug xerces-c_2D optimized xerces-c_2 rdTools rdSimulation)

ADD_EXECUTABLE(storageFilter storageFilter.cpp)
TARGET_LINK_LIBRARIES(storageFilter debug xerces-c_2D optimized xerces-c_2 rdTools)

INSTALL_TARGETS(/bin/ convertControls)
INSTALL_TARGETS(/bin/ convertStorageToControls)
INSTALL_TARGETS(/bin/ simplifyControls)
INSTALL_TARGETS(/bin/ perturbStorage)
INSTALL_TARGETS(/bin/ storageAdd)
INSTALL_TARGETS(/bin/ storageIntegrate)
INSTALL_TARGETS(/bin/ storageInterpolate)
INSTALL_TARGETS(/bin/ storageSubtract)
INSTALL_TARGETS(/bin/ storageFilter)
ENDIF(NOT OPENSIM_MINIMAL_BUILD)

SUBDIRS(simmToOpenSim addComments)
