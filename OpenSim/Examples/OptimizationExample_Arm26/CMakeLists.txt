cmake_minimum_required(VERSION 2.6)

PROJECT (OptimizerMain_Arm26)

FILE(GLOB SOURCE_FILES *.cpp )
FILE(GLOB INCLUDE_FILES *.h )
FILE(GLOB OSIM_FILES *.osim ) 

#
# Copy files to run dir
#
FOREACH (dataFile ${OSIM_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET optimizerExample_Arm26
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${optimizerMain_Arm26_BINARY_DIR})
ENDFOREACH (dataFile)

SET(SOURCE_FILES OptimizationExample.cpp) 

SET(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release"
      CACHE STRING "Semicolon separated list of supported configuration types, only supports Debug, Release, MinSizeRel, and RelWithDebInfo, anything else will be ignored." FORCE )

SET(OPENSIM_INSTALL_DIR "C:/Program Files/OpenSim 2.0" 
		CACHE PATH "Top-level directory of OpenSim install")

# Location of headers
SET(SIMTK_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include/SimTK/include)
SET(OPENSIM_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include)
INCLUDE_DIRECTORIES(${SIMTK_HEADERS_DIR} ${OPENSIM_HEADERS_DIR})

# Libraries and dlls
SET(OPENSIM_LIBS_DIR ${OPENSIM_INSTALL_DIR}/sdk/lib)
SET(OPENSIM_DLLS_DIR ${OPENSIM_INSTALL_DIR}/bin)
LINK_DIRECTORIES(${OPENSIM_LIBS_DIR} ${OPENSIM_DLLS_DIR})


SET(NameSpace "OpenSim_" CACHE STRING "Prefix for simtk lib names, includes trailing '_'. Leave empty to use stock SimTK libraries.")
MARK_AS_ADVANCED(NameSpace)

LINK_LIBRARIES(
	debug osimCommon_d optimized osimCommon
	debug osimSimulation_d optimized osimSimulation
	debug osimAnalyses_d optimized osimAnalyses
	debug osimActuators_d optimized osimActuators
	debug osimTools_d optimized osimTools
	debug ${NameSpace}SimTKcommon_d optimized   ${NameSpace}SimTKcommon
	debug ${NameSpace}SimTKmath_d optimized  ${NameSpace}SimTKmath
	debug ${NameSpace}SimTKcpodes_d optimized  ${NameSpace}SimTKcpodes
	debug ${NameSpace}SimTKsimbody_d optimized  ${NameSpace}SimTKsimbody
	SimTKlapack pthreadVC2)

ADD_EXECUTABLE(optimizerExample_Arm26  ${SOURCE_FILES} ${INCLUDE_FILES}) 

#IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	SET(CMAKE_INSTALL_PREFIX ${OptimizerMain_Arm26_SOURCE_DIR})
#ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)

SET_TARGET_PROPERTIES(
	optimizerExample_Arm26 PROPERTIES
	PROJECT_LABEL "OptimizerExample_Arm26")

INSTALL_TARGETS(/bin optimizerExample_Arm26)

# From here on we're just copying data and model files to run directory
ADD_CUSTOM_TARGET(copyDataFiles)

FILE(GLOB DATA_FILES *.osim *.xml *.sto *.mot *.vtp *.obj)
#
# Copy files to run dir
#
FOREACH (dataFile ${DATA_FILES})
 GET_FILENAME_COMPONENT(DATAFILE ${dataFile} NAME)
 ADD_CUSTOM_COMMAND(
    TARGET copyDataFiles
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OptimizerMain_Arm26_BINARY_DIR}/${DATAFILE})
ENDFOREACH (dataFile) 


ADD_DEPENDENCIES(optimizerExample_Arm26 copyDataFiles)