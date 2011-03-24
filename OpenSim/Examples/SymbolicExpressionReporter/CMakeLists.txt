Project(ExpressionReporterPlugin)

cmake_minimum_required(VERSION 2.6)
if(COMMAND cmake_policy)
        cmake_policy(SET CMP0003 NEW)
        cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)

SET(KIT plugin)
SET(UKIT PLUGIN)

SET(NameSpace "OpenSim_")

FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB INCLUDE_FILES *.h)

SET(EXPORT_MACRO OSIMEXPPLUGIN_EXPORTS)

##SET(OPENSIM_INSTALL_DIR "G:/OpenSim22/" CACHE PATH "Top-level directory of OpenSim install")

SET(PLUGIN_NAME "osimExpressionReporter")

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
${OpenSim_SOURCE_DIR}/OpenSim ${OpenSim_SOURCE_DIR}/Vendors/lepton/include ${XERCES_HEADER_DIR} ${SIMTK_HEADERS_DIR})

### LIBRARIES
SET(OPENSIM_LIBS_DIR  ${OPENSIM_INSTALL_DIR}/sdk/lib)
LINK_DIRECTORIES(${OPENSIM_LIBS_DIR})

# SimmKinematicsEngine library only needed for SimmFileWriter...
LINK_LIBRARIES(
	debug osimCommon_d optimized osimCommon
	debug osimSimulation_d optimized osimSimulation
	debug osimAnalyses_d optimized osimAnalyses
	debug osimActuators_d optimized osimActuators
	debug osimTools_d optimized osimTools
	debug  ${NameSpace}SimTKcommon_d optimized   ${NameSpace}SimTKcommon
	debug  ${NameSpace}SimTKmath_d optimized  ${NameSpace}SimTKmath
	debug ${NameSpace}SimTKsimbody_d optimized  ${NameSpace}SimTKsimbody
	debug osimLepton_d optimized osimLepton
	SimTKlapack pthreadVC2
)

ADD_LIBRARY(${PLUGIN_NAME} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	SET(CMAKE_INSTALL_PREFIX ${OPENSIM_INSTALL_DIR}/ CACHE PATH "Install path prefix." FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

## MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
## MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
## MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)

SET_TARGET_PROPERTIES(
	${PLUGIN_NAME} PROPERTIES
	DEFINE_SYMBOL ${EXPORT_MACRO}
	PROJECT_LABEL "Libraries - ${PLUGIN_NAME}")

IF(WIN32)
	INSTALL_TARGETS(/sdk/lib/ RUNTIME_DIRECTORY /bin ${PLUGIN_NAME})
ELSE(WIN32)
	INSTALL_TARGETS(/bin/ RUNTIME_DIRECTORY /bin ${PLUGIN_NAME})
ENDIF(WIN32)

