###########################################
## For building with the rest of OpenSim ##
###########################################
IF(BUILD_API_EXAMPLES)

SET(EXAMPLE_MAIN exampleMain)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		  
		    ${XERCES_INCLUDE_DIR})

ADD_EXECUTABLE(${EXAMPLE_MAIN} TugOfWar_Complete.cpp)
TARGET_LINK_LIBRARIES(${EXAMPLE_MAIN}
	debug osimSimulation${CMAKE_DEBUG_POSTFIX}	optimized osimSimulation
	debug osimActuators${CMAKE_DEBUG_POSTFIX}	optimized osimActuators
	debug osimCommon${CMAKE_DEBUG_POSTFIX}		optimized osimCommon
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX}	optimized osimAnalyses
	debug osimTools${CMAKE_DEBUG_POSTFIX}		optimized osimTools
	${SIMTK_ALL_LIBS}
)
SET_TARGET_PROPERTIES(${EXAMPLE_MAIN}
	PROPERTIES
	PROJECT_LABEL "Examples - ${EXAMPLE_MAIN}"
)

FILE(GLOB DATA_FILES *.osim *.xml *.sto *.mot *.vtp *.obj)
FOREACH (DATA_FILE ${DATA_FILES})
 ADD_CUSTOM_COMMAND(
	TARGET ${EXAMPLE_MAIN}
	COMMAND ${CMAKE_COMMAND}
	ARGS -E copy
	${DATA_FILE}
	${OpenSim_BINARY_DIR}/OpenSim/Examples/ExampleMain
  )
ENDFOREACH (DATA_FILE)

IF(BUILD_TESTING)
  SET(TEST_EXAMPLE_MAIN testExampleMain)

  ADD_EXECUTABLE(${TEST_EXAMPLE_MAIN}Output testExampleMain.cpp)
  TARGET_LINK_LIBRARIES(${TEST_EXAMPLE_MAIN}Output
	debug osimSimulation${CMAKE_DEBUG_POSTFIX}	optimized osimSimulation
	debug osimActuators${CMAKE_DEBUG_POSTFIX}	optimized osimActuators
	debug osimCommon${CMAKE_DEBUG_POSTFIX}		optimized osimCommon
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX}	optimized osimAnalyses
	debug osimTools${CMAKE_DEBUG_POSTFIX}		optimized osimTools
	${SIMTK_ALL_LIBS}
  )
  SET_TARGET_PROPERTIES(${TEST_EXAMPLE_MAIN}Output
	PROPERTIES 
	PROJECT_LABEL "Tests - ${TEST_EXAMPLE_MAIN}Output"
  )

  ADD_TEST(${TEST_EXAMPLE_MAIN}Runs ${EXECUTABLE_OUTPUT_PATH}/${EXAMPLE_MAIN})
  ADD_TEST(${TEST_EXAMPLE_MAIN}Output ${EXECUTABLE_OUTPUT_PATH}/${TEST_EXAMPLE_MAIN}Output)
ENDIF(BUILD_TESTING)

################################################
## For building with existing OpenSim install ##
################################################
ELSE(BUILD_API_EXAMPLES)

cmake_minimum_required(VERSION 2.6)

PROJECT (OpenSimTugOfWar)

# Change this to TugOfWar_Complete.cpp if you want to 
# run the whole example, or you can change the source
# file from within Visual Studio.

SET(TARGET TugOfWar1_CreateModel
		CACHE STRING "Name of example to build")
SET(SOURCE_FILES ${TARGET}.cpp) 

SET(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release"
      CACHE STRING "Semicolon separated list of supported configuration types, only supports Debug, Release, MinSizeRel, and RelWithDebInfo, anything else will be ignored." FORCE )

SET(OPENSIM_INSTALL_DIR "C:/OpenSim2.4.0" 
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

ADD_EXECUTABLE(${TARGET}  ${SOURCE_FILES}) 

#IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#	SET(CMAKE_INSTALL_PREFIX ${EXECUTABLE_OUTPUT_PATH}
#			CACHE STRING "Install path prefix." FORCE)
#ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)

SET_TARGET_PROPERTIES(
	${TARGET} PROPERTIES
	PROJECT_LABEL "Example Main")

#INSTALL_TARGETS(/bin ${TARGET})


#
# Copy files to run dir
#
FILE(GLOB DATA_FILES *.vtp *.obj)
ADD_CUSTOM_TARGET(copyRequiredFiles)
ADD_DEPENDENCIES(${TARGET} copyRequiredFiles)
FOREACH (dataFile ${DATA_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET copyRequiredFiles
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSimTugOfWar_BINARY_DIR})
 ENDFOREACH (dataFile) 

ENDIF(BUILD_API_EXAMPLES)