INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors ${XERCES_INCLUDE_DIR})
LINK_LIBRARIES(debug osimCommon${CMAKE_DEBUG_POSTFIX}     optimized osimCommon 
               debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
               debug osimActuators${CMAKE_DEBUG_POSTFIX}  optimized osimActuators)

FOREACH(TARGET ${TARGETS})
	ADD_EXECUTABLE(${TARGET} ${TARGET}.cpp)
	SET_TARGET_PROPERTIES(${TARGET} PROPERTIES PROJECT_LABEL "Utilities - ${TARGET}")
ENDFOREACH(TARGET)


IF(BUILD_UTILITIES)
	SUBDIRS(addComments simmFileWriterDLL simmToOpenSim) ## openSimToSimm 
	INSTALL_TARGETS(/sdk/bin/ ${TARGETS})
	SET(OLD_VERSION_IMPORTER_FILES "importOldModels/migrate15to18.exe"   "importOldModels/xerces-c_2_7.dll" )
	INSTALL_FILES(/bin/ FILES  ${OpenSim_SOURCE_DIR}/OpenSim/Utilities/${OLD_VERSION_IMPORTER_FILES})
ELSE(BUILD_UTILITIES)
	SUBDIRS(simmFileWriterDLL)
ENDIF(BUILD_UTILITIES)