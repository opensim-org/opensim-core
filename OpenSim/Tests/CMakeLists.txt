cmake_minimum_required(VERSION 2.6)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors
		${XERCES_INCLUDE_DIR})

IF(WIN32)
  SET( THREAD_LIB pthreadVC2)
ELSE(WIN32)
  SET( THREAD_LIB )
ENDIF(WIN32)

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
   	SimTKlapack ${THREAD_LIB})

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	SET(CMAKE_INSTALL_PREFIX ${OPENSIM_INSTALL_DIR})
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)


# Automatically create tests using files named "test*.cpp"	
FILE(GLOB TEST_PROGS "test*.cpp")

FOREACH(TEST_PROG ${TEST_PROGS})
    GET_FILENAME_COMPONENT(TEST_ROOT ${TEST_PROG} NAME_WE)

    # Link with shared library
    ADD_EXECUTABLE(${TEST_ROOT} ${TEST_PROG})
    TARGET_LINK_LIBRARIES(${TEST_ROOT} ${LINK_LIBRARIES})
	ADD_TEST(${TEST_ROOT} ${EXECUTABLE_OUTPUT_PATH}/${TEST_ROOT} 1)
    SET_TARGET_PROPERTIES(${TEST_ROOT} 
		PROPERTIES 
		PROJECT_LABEL "Tests - ${TEST_ROOT}")

ENDFOREACH(TEST_PROG ${TEST_PROGS})

# Also need to copy data files (.osim, .xml, .sto, .trc, .mot) to run directory
FILE(GLOB TEST_FILES *.osim *.obj *.vtp *.xml *.sto *.mot)
#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
  ADD_CUSTOM_COMMAND(
     TARGET testOpenSimAPI
     COMMAND ${CMAKE_COMMAND}
     ARGS -E copy
     ${dataFile}
     ${OpenSim_BINARY_DIR}/OpenSim/Tests)
 
ENDFOREACH (dataFile) 

INSTALL_TARGETS(/bin testOpenSimAPI)

SUBDIRS(Wrapping)
