#INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
#		    ${OpenSim_SOURCE_DIR}/Vendors)


LINK_LIBRARIES( debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
		debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
		debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
		debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
		debug osimLepton${CMAKE_DEBUG_POSTFIX} optimized osimLepton
		${SIMTK_ALL_LIBS})

ADD_CUSTOM_TARGET(copyTestFiles)

FILE(GLOB TEST_PROGS "test*.cpp")

FOREACH(TEST_PROG ${TEST_PROGS})
    GET_FILENAME_COMPONENT(TEST_ROOT ${TEST_PROG} NAME_WE)

    # Link with shared library
    ADD_EXECUTABLE(${TEST_ROOT} ${TEST_PROG})
    TARGET_LINK_LIBRARIES(${TEST_ROOT} ${LINK_LIBRARIES} )
    ADD_TEST(${TEST_ROOT} ${EXECUTABLE_OUTPUT_PATH}/${TEST_ROOT})
    SET_TARGET_PROPERTIES(${TEST_ROOT} 
		PROPERTIES 
		PROJECT_LABEL "Tests - ${TEST_ROOT}")

ENDFOREACH(TEST_PROG ${TEST_PROGS})

#ADD_EXECUTABLE(testPrescribedForce testPrescribedForce.cpp)

FILE(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.obj)

#
# Copy files to run dir
#
ADD_DEPENDENCIES(${TEST_ROOT} copyTestFiles)
FOREACH (dataFile ${TEST_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET copyTestFiles
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/OpenSim/Simulation/Test)
 ENDFOREACH (dataFile) 

#
# Testing
#

#IF (UNIX)
#  ADD_DEFINITIONS(-fprofile-arcs -ftest-coverage)
#  LINK_LIBRARIES(gcov)
#ENDIF (UNIX)

#ENABLE_TESTING()

#IF (EXECUTABLE_OUTPUT_PATH)
#  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
#ELSE (EXECUTABLE_OUTPUT_PATH)
#  SET (TEST_PATH .)
#ENDIF (EXECUTABLE_OUTPUT_PATH)

#ADD_TEST(testPrescribedForce ${TEST_PATH}/testPrescribedForce)
