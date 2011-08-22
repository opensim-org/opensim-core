###########################################
## For building with the rest of OpenSim ##
###########################################
IF(BUILD_API_EXAMPLES)

SET(EXAMPLE_CUSTOM_ACTUATOR exampleCustomActuator)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		  
		    ${XERCES_INCLUDE_DIR})

ADD_EXECUTABLE(${EXAMPLE_CUSTOM_ACTUATOR} PistonActuator.cpp toyLeg_example.cpp PistonActuator.h)
TARGET_LINK_LIBRARIES(${EXAMPLE_CUSTOM_ACTUATOR}
	debug osimSimulation${CMAKE_DEBUG_POSTFIX}	optimized osimSimulation
	debug osimActuators${CMAKE_DEBUG_POSTFIX}	optimized osimActuators
	debug osimCommon${CMAKE_DEBUG_POSTFIX}		optimized osimCommon
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX}	optimized osimAnalyses
	debug osimTools${CMAKE_DEBUG_POSTFIX}		optimized osimTools
	${SIMTK_ALL_LIBS}
)
SET_TARGET_PROPERTIES(${EXAMPLE_CUSTOM_ACTUATOR}
	PROPERTIES
	PROJECT_LABEL "Examples - ${EXAMPLE_CUSTOM_ACTUATOR}"
)

FILE(GLOB DATA_FILES *.osim *.xml *.sto *.mot *.vtp *.obj)
FOREACH (DATA_FILE ${DATA_FILES})
 ADD_CUSTOM_COMMAND(
	TARGET ${EXAMPLE_CUSTOM_ACTUATOR}
	COMMAND ${CMAKE_COMMAND}
	ARGS -E copy
	${DATA_FILE}
	${OpenSim_BINARY_DIR}/OpenSim/Examples/CustomActuatorExample
  )
ENDFOREACH (DATA_FILE)

IF(BUILD_TESTING)
  SET(TEST_EXAMPLE_CUSTOM_ACTUATOR testExampleCustomActuator)

  ADD_EXECUTABLE(${TEST_EXAMPLE_CUSTOM_ACTUATOR}Output testCustomActuatorExample.cpp)
  TARGET_LINK_LIBRARIES(${TEST_EXAMPLE_CUSTOM_ACTUATOR}Output
	debug osimSimulation${CMAKE_DEBUG_POSTFIX}	optimized osimSimulation
	debug osimActuators${CMAKE_DEBUG_POSTFIX}	optimized osimActuators
	debug osimCommon${CMAKE_DEBUG_POSTFIX}		optimized osimCommon
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX}	optimized osimAnalyses
	debug osimTools${CMAKE_DEBUG_POSTFIX}		optimized osimTools
	${SIMTK_ALL_LIBS}
  )
  SET_TARGET_PROPERTIES(${TEST_EXAMPLE_CUSTOM_ACTUATOR}Output
	PROPERTIES 
	PROJECT_LABEL "Tests - ${TEST_EXAMPLE_CUSTOM_ACTUATOR}Output"
  )

  ADD_TEST(${TEST_EXAMPLE_CUSTOM_ACTUATOR}Runs ${EXECUTABLE_OUTPUT_PATH}/${EXAMPLE_CUSTOM_ACTUATOR})
  ADD_TEST(${TEST_EXAMPLE_CUSTOM_ACTUATOR}Output ${EXECUTABLE_OUTPUT_PATH}/${TEST_EXAMPLE_CUSTOM_ACTUATOR}Output)
ENDIF(BUILD_TESTING)

################################################
## For building with existing OpenSim install ##
################################################
ELSE(BUILD_API_EXAMPLES)

cmake_minimum_required(VERSION 2.6)

PROJECT (CustomActuatorExample)

FILE(GLOB SOURCE_FILES *.cpp )
FILE(GLOB INCLUDE_FILES *.h ) 

SET(TARGET toyLeg_example
		CACHE STRING "Name of example to build")

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

SET(NameSpace "OpenSim_" CACHE STRING "Prefix for simtk lib names, includes trailing _. Leave empty to use stock SimTK libraries.")
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

ADD_EXECUTABLE(${TARGET}  ${SOURCE_FILES} ${INCLUDE_FILES}) 

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	SET(CMAKE_INSTALL_PREFIX ${CustomActuatorExample_SOURCE_DIR}
			CACHE STRING "Install path prefix." FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)

SET_TARGET_PROPERTIES(
	${TARGET} PROPERTIES
	PROJECT_LABEL "Actuator_examples")

INSTALL_TARGETS(/ ${TARGET})

ENDIF(BUILD_API_EXAMPLES)