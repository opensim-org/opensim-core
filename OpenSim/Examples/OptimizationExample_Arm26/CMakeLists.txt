###########################################
## For building with the rest of OpenSim ##
###########################################
IF(BUILD_API_EXAMPLES)

SET(EXAMPLE_OPTIMIZATION exampleOptimization)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		  
		    ${XERCES_INCLUDE_DIR})

ADD_EXECUTABLE(${EXAMPLE_OPTIMIZATION} OptimizationExample.cpp)
TARGET_LINK_LIBRARIES(${EXAMPLE_OPTIMIZATION}
	debug osimSimulation${CMAKE_DEBUG_POSTFIX}	optimized osimSimulation
	debug osimActuators${CMAKE_DEBUG_POSTFIX}	optimized osimActuators
	debug osimCommon${CMAKE_DEBUG_POSTFIX}		optimized osimCommon
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX}	optimized osimAnalyses
	debug osimTools${CMAKE_DEBUG_POSTFIX}		optimized osimTools
	${SIMTK_ALL_LIBS}
)
SET_TARGET_PROPERTIES(${EXAMPLE_OPTIMIZATION}
	PROPERTIES
	PROJECT_LABEL "Examples - ${EXAMPLE_OPTIMIZATION}"
)

FILE(GLOB DATA_FILES *.osim *.xml *.sto *.mot *.vtp *.obj)
FOREACH (DATA_FILE ${DATA_FILES})
 ADD_CUSTOM_COMMAND(
	TARGET ${EXAMPLE_OPTIMIZATION}
	COMMAND ${CMAKE_COMMAND}
	ARGS -E copy
	${DATA_FILE}
	${OpenSim_BINARY_DIR}/OpenSim/Examples/OptimizationExample_Arm26
  )
ENDFOREACH (DATA_FILE)

IF(BUILD_TESTING)
  SET(TEST_EXAMPLE_OPTIMIZATION testExampleOptimization)

  ADD_EXECUTABLE(${TEST_EXAMPLE_OPTIMIZATION}Output testOptimizationExample.cpp)
  TARGET_LINK_LIBRARIES(${TEST_EXAMPLE_OPTIMIZATION}Output
	debug osimSimulation${CMAKE_DEBUG_POSTFIX}	optimized osimSimulation
	debug osimActuators${CMAKE_DEBUG_POSTFIX}	optimized osimActuators
	debug osimCommon${CMAKE_DEBUG_POSTFIX}		optimized osimCommon
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX}	optimized osimAnalyses
	debug osimTools${CMAKE_DEBUG_POSTFIX}		optimized osimTools
	${SIMTK_ALL_LIBS}
  )
  SET_TARGET_PROPERTIES(${TEST_EXAMPLE_OPTIMIZATION}Output
	PROPERTIES 
	PROJECT_LABEL "Tests - ${TEST_EXAMPLE_OPTIMIZATION}Output"
  )

  ADD_TEST(${TEST_EXAMPLE_OPTIMIZATION}Runs ${EXECUTABLE_OUTPUT_PATH}/${EXAMPLE_OPTIMIZATION})
  ADD_TEST(${TEST_EXAMPLE_OPTIMIZATION}Output ${EXECUTABLE_OUTPUT_PATH}/${TEST_EXAMPLE_OPTIMIZATION}Output)
ENDIF(BUILD_TESTING)

################################################
## For building with existing OpenSim install ##
################################################
ELSE(BUILD_API_EXAMPLES)

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

SET(OPENSIM_INSTALL_DIR "C:/OpenSim2.3.1" 
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

IF(WIN32)
	SET(PLATFORM_LIBS  pthreadVC2)
ELSE (WIN32)
  SET(NameSpace "")
  IF(APPLE)
	SET(PLATFORM_LIBS  SimTKAtlas)
  ELSE(APPLE)
	SET(PLTAFORM_LIBS SimTKAtlas_Lin_generic)
  ENDIF(APPLE)
ENDIF (WIN32)

LINK_LIBRARIES(
	debug osimCommon_d optimized osimCommon
	debug osimSimulation_d optimized osimSimulation
	debug osimAnalyses_d optimized osimAnalyses
	debug osimActuators_d optimized osimActuators
	debug osimTools_d optimized osimTools
	debug ${NameSpace}SimTKcommon_d optimized ${NameSpace}SimTKcommon
	debug ${NameSpace}SimTKmath_d optimized  ${NameSpace}SimTKmath
	debug ${NameSpace}SimTKsimbody_d optimized ${NameSpace}SimTKsimbody
	SimTKlapack
	${PLATFORM_LIBS})

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

ENDIF(BUILD_API_EXAMPLES)