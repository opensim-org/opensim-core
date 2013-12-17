Project(BodyDragForce)

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

SET(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release"
      CACHE STRING "Semicolon separated list of supported configuration types, only supports Debug, Release, MinSizeRel, and RelWithDebInfo, anything else will be ignored." FORCE )

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

SET(OPENSIM_INSTALL_DIR "C:/Program Files (x86)/OpenSim 3.0/" CACHE PATH 
    "Top-level directory of OpenSim install")

SET(PLUGIN_NAME "BodyDragForce" CACHE STRING "Name of shared library to create")

### HEADERS
SET(OPENSIM_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include)
SET(SIMTK_HEADERS_DIR   ${OPENSIM_INSTALL_DIR}/sdk/include/SimTK/include)

INCLUDE_DIRECTORIES(${OPENSIM_HEADERS_DIR} 
                    ${SIMTK_HEADERS_DIR})

### LIBRARIES
SET(OPENSIM_LIBS_DIR  
    ${OPENSIM_INSTALL_DIR}/lib
    ${OPENSIM_INSTALL_DIR}/sdk/lib)
LINK_DIRECTORIES(${OPENSIM_LIBS_DIR})

# Assumes Windows
LINK_LIBRARIES(
	debug osimCommon_d     optimized osimCommon
	debug osimSimulation_d optimized osimSimulation
	debug osimAnalyses_d   optimized osimAnalyses
	debug osimActuators_d  optimized osimActuators
	debug osimTools_d      optimized osimTools
	debug     ${NameSpace}SimTKcommon_d
    optimized ${NameSpace}SimTKcommon
	debug     ${NameSpace}SimTKmath_d
    optimized ${NameSpace}SimTKmath
	debug     ${NameSpace}SimTKsimbody_d
    optimized ${NameSpace}SimTKsimbody
    pthreadVC2
)

ADD_LIBRARY(${PLUGIN_NAME} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	SET(CMAKE_INSTALL_PREFIX ${OPENSIM_INSTALL_DIR}/ CACHE PATH 
        "Install path prefix." FORCE)
ENDIF()

MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)

SET_TARGET_PROPERTIES(
	${PLUGIN_NAME} PROPERTIES
	DEFINE_SYMBOL ${EXPORT_MACRO}
	PROJECT_LABEL "Libraries - ${PLUGIN_NAME}")

INSTALL_TARGETS(/plugins RUNTIME_DIRECTORY /plugins  ${PLUGIN_NAME})

