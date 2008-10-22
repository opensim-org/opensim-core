SET(TARGETS convertControls convertStorageToControls simplifyControls perturbStorage 
				storageAdd storageIntegrate storageInterpolate storageSubtract storageFilter)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors ${XERCES_INCLUDE_DIR})
LINK_LIBRARIES(debug xerces-c_2D optimized xerces-c_2 osimCommon osimSimulation)

FOREACH(TARGET ${TARGETS})
	ADD_EXECUTABLE(${TARGET} ${TARGET}.cpp)
	SET_TARGET_PROPERTIES(${TARGET} PROPERTIES PROJECT_LABEL "Utilities - ${TARGET}")
ENDFOREACH(TARGET)

INSTALL_TARGETS(/sdk/bin/ ${TARGETS})
SET_TARGET_PROPERTIES(${TARGETS} PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD})

SUBDIRS(simmToOpenSim openSimToSimm addComments convertFiles)
