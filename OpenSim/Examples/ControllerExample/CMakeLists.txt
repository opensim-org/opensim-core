cmake_minimum_required(VERSION 2.6)

PROJECT (ControllerExample)

SET(SOURCE_FILES ControllerExample.cpp) 

SET(OPENSIM_INSTALL_DIR "C:/OpenSim2.3.1" CACHE PATH "Top-level directory of OpenSim install")

SET(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release"
      CACHE STRING "Semicolon separated list of supported configuration types, only supports Debug, Release, MinSizeRel, and RelWithDebInfo, anything else will be ignored." FORCE )

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
	debug ${NameSpace}SimTKsimbody_d optimized  ${NameSpace}SimTKsimbody
	SimTKlapack pthreadVC2)

ADD_EXECUTABLE(main  ${SOURCE_FILES} ) 

#IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	SET(CMAKE_INSTALL_PREFIX ${OPENSIM_INSTALL_DIR})
#ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)


SET_TARGET_PROPERTIES(
	main PROPERTIES
	PROJECT_LABEL "Controller Example")

INSTALL_TARGETS(/ main)

ADD_CUSTOM_TARGET(copyDataFiles)

FILE(GLOB DATA_FILES *.osim *.xml *.sto *.mot *.vtp *.obj)
#
# Copy files to run dir
#
FOREACH (dataFile ${DATA_FILES})
 # MESSAGE( "ControllerExample_SOURCE_DIR file "${ControllerExample_SOURCE_DIR})
 # MESSAGE( "datafile file "${dataFile})
 GET_FILENAME_COMPONENT(DATAFILE ${dataFile} NAME)
 ADD_CUSTOM_COMMAND(
    TARGET copyDataFiles
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${ControllerExample_BINARY_DIR}/${DATAFILE})
ENDFOREACH (dataFile) 


ADD_DEPENDENCIES(main copyDataFiles)
