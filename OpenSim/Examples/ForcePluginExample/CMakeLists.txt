###########################################
## For building with the rest of OpenSim ##
###########################################
IF(BUILD_API_EXAMPLES)

SET(EXAMPLE_FORCE_PLUGIN exampleForcePlugin)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR}
		    ${OpenSim_SOURCE_DIR}/Vendors	  
		    ${XERCES_INCLUDE_DIR})

ADD_LIBRARY(${EXAMPLE_FORCE_PLUGIN} SHARED CoupledBushingForce.cpp RegisterTypes_osimPlugin.cpp CoupledBushingForce.h osimPluginDLL.h RegisterTypes_osimPlugin.h)
TARGET_LINK_LIBRARIES(${EXAMPLE_FORCE_PLUGIN}
	debug osimSimulation${CMAKE_DEBUG_POSTFIX}	optimized osimSimulation
	debug osimActuators${CMAKE_DEBUG_POSTFIX}	optimized osimActuators
	debug osimCommon${CMAKE_DEBUG_POSTFIX}		optimized osimCommon
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX}	optimized osimAnalyses
	debug osimTools${CMAKE_DEBUG_POSTFIX}		optimized osimTools
	${SIMTK_ALL_LIBS}
)
SET_TARGET_PROPERTIES(${EXAMPLE_FORCE_PLUGIN}
	PROPERTIES
	DEFINE_SYMBOL OSIMPLUGIN_EXPORTS
	PROJECT_LABEL "Examples - ${EXAMPLE_FORCE_PLUGIN}"
	RUNTIME_OUTPUT_DIRECTORY ${OpenSim_BINARY_DIR}/OpenSim/Examples/ForcePluginExample
)

################################################
## For building with existing OpenSim install ##
################################################
ELSE(BUILD_API_EXAMPLES)

Project(CoupledBushingForcePlugin)

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

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

SET(OPENSIM_INSTALL_DIR "C:/OpenSim2.3.1/" CACHE PATH "Top-level directory of OpenSim install")

SET(PLUGIN_NAME "osimCoupledBushingForcePlugin" CACHE STRING "Name of shared library to create")

### HEADERS
SET(OPENSIM_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include)
SET(XERCES_HEADER_DIR ${OPENSIM_INSTALL_DIR}/sdk/include/xercesc)
SET(SIMTK_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include/SimTK/include)

INCLUDE_DIRECTORIES(${OPENSIM_HEADERS_DIR} ${XERCES_HEADER_DIR} ${SIMTK_HEADERS_DIR})

### LIBRARIES
SET(OPENSIM_LIBS_DIR  ${OPENSIM_INSTALL_DIR}/sdk/lib)
LINK_DIRECTORIES(${OPENSIM_LIBS_DIR})

# SimmKinematicsEngine library only needed for SimmFileWriter...
LINK_LIBRARIES(
	debug osimCommon_d optimized osimCommon
	debug osimSimulation_d optimized osimSimulation
	debug osimAnalyses_d optimized osimAnalyses
	debug osimActuators_d optimized osimActuators
	debug  ${NameSpace}SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized   ${NameSpace}SimTKcommon
	debug  ${NameSpace}SimTKmath${CMAKE_DEBUG_POSTFIX} optimized  ${NameSpace}SimTKmath
	debug ${NameSpace}SimTKsimbody${CMAKE_DEBUG_POSTFIX} optimized  ${NameSpace}SimTKsimbody
	SimTKlapack pthreadVC2
)

ADD_LIBRARY(${PLUGIN_NAME} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	SET(CMAKE_INSTALL_PREFIX ${OPENSIM_INSTALL_DIR}/ CACHE PATH "Install path prefix." FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

## MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)

SET_TARGET_PROPERTIES(
	${PLUGIN_NAME} PROPERTIES
	DEFINE_SYMBOL ${EXPORT_MACRO}
	PROJECT_LABEL "Libraries - ${PLUGIN_NAME}")

INSTALL_TARGETS(/plugins RUNTIME_DIRECTORY /plugins  ${PLUGIN_NAME})

ENDIF(BUILD_API_EXAMPLES)