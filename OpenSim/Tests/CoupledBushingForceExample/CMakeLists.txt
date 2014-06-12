cmake_minimum_required(VERSION 2.6)

# Variable definitions
SET(EXAMPLE_TARGET exampleCoupledBushingForcePlugin)
SET(EXAMPLE_DIR ${OpenSim_SOURCE_DIR}/OpenSim/Examples/Plugins/CoupledBushingForceExample)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors)
			
FILE(GLOB SOURCE_FILES ${EXAMPLE_DIR}/*.h ${EXAMPLE_DIR}/*.cpp)
ADD_LIBRARY(${EXAMPLE_TARGET} SHARED ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(${EXAMPLE_TARGET}
	debug osimSimulation${CMAKE_DEBUG_POSTFIX}	optimized osimSimulation
	debug osimActuators${CMAKE_DEBUG_POSTFIX}	optimized osimActuators
	debug osimCommon${CMAKE_DEBUG_POSTFIX}		optimized osimCommon
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX}	optimized osimAnalyses
	debug osimTools${CMAKE_DEBUG_POSTFIX}		optimized osimTools
	${SIMTK_ALL_LIBS}
)
SET_TARGET_PROPERTIES(${EXAMPLE_TARGET}
	PROPERTIES
	DEFINE_SYMBOL OSIMPLUGIN_EXPORTS
	PROJECT_LABEL "Examples - ${EXAMPLE_TARGET}"
	RUNTIME_OUTPUT_DIRECTORY ${OpenSim_BINARY_DIR}/OpenSim/Examples/Plugins/CoupledBushingForceExample
)