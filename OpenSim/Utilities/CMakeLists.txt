SET(TARGETS convertControls convertStorageToControls simplifyControls perturbStorage 
				storageAdd storageIntegrate storageInterpolate storageSubtract storageFilter openSimToSimm migrateSimmKEModel migrate15to18)

IF (BUILD_STATIC_OSIM_LIBS)
	ADD_DEFINITIONS(-DSTATIC_OSIM_LIBS)
ENDIF(BUILD_STATIC_OSIM_LIBS)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors ${XERCES_INCLUDE_DIR})
LINK_LIBRARIES(debug xerces-c_2D optimized xerces-c_2 
				debug osimCommon_d optimized osimCommon
				debug osimSimulation_d optimized osimSimulation)

FOREACH(TARGET ${TARGETS})
	ADD_EXECUTABLE(${TARGET} ${TARGET}.cpp)
	SET_TARGET_PROPERTIES(${TARGET} PROPERTIES PROJECT_LABEL "Utilities - ${TARGET}")
ENDFOREACH(TARGET)

TARGET_LINK_LIBRARIES(openSimToSimm osimSimmFileWriter)
TARGET_LINK_LIBRARIES(migrateSimmKEModel osimActuators osimSimbodyEngine osimSimmKinematicsEngine osimMigrateSimmKEModel)
TARGET_LINK_LIBRARIES(migrate15to18 osimActuators osimSimbodyEngine osimSimmKinematicsEngine osimMigrateSimmKEModel)

INSTALL_TARGETS(/sdk/bin/ ${TARGETS})
SET_TARGET_PROPERTIES(${TARGETS} PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD})

SUBDIRS(simmToOpenSim addComments convertFiles simmFileWriterDLL migrateSimmKEModelDLL)
