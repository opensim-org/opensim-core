PROJECT (OpenSimMain)

SET(SOURCE_FILES main.cpp) 

SET(OPENSIM_INSTALL_DIR "C:/Program Files/OpenSim 1.6/" CACHE PATH "Top-level directory of OpenSim install")

### HEADERS
SET(OPENSIM_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include)
SET(XERCES_HEADER_DIR ${OPENSIM_INSTALL_DIR}/sdk/include/xercesc)
SET(SIMTK_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include/SimTK/include)

INCLUDE_DIRECTORIES(${OPENSIM_HEADERS_DIR} ${XERCES_HEADER_DIR} ${SIMTK_HEADERS_DIR})

### Executables
SET(OPENSIM_LIBS_DIR  ${OPENSIM_INSTALL_DIR}/sdk/lib)
LINK_DIRECTORIES(${OPENSIM_LIBS_DIR})


LINK_LIBRARIES(
	debug osimCommon_d optimized osimCommon
	debug osimSimulation_d optimized osimSimulation
	debug osimAnalyses_d optimized osimAnalyses
	debug osimActuators_d optimized osimActuators
	debug osimSimbodyEngine_d optimized osimSimbodyEngine
	debug  SimTKcommon_d optimized   SimTKcommon
	debug  SimTKmath_d optimized  SimTKmath
	debug SimTKcpodes_d optimized  SimTKcpodes
	debug SimTKsimbody_d optimized  SimTKsimbody
	SimTKlapack pthreadVC2)

ADD_EXECUTABLE(main  ${SOURCE_FILES} ) 

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	SET(CMAKE_INSTALL_PREFIX ${OPENSIM_INSTALL_DIR})
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

SET_TARGET_PROPERTIES(
	main PROPERTIES
	PROJECT_LABEL "Example Main")

INSTALL_TARGETS(/bin main )

