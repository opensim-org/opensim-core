INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors)


LINK_LIBRARIES( debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
		debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
		debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        )

ADD_EXECUTABLE(testForward testForward.cpp)

FILE(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET testForward
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/Applications/Forward/test)
 ENDFOREACH (dataFile) 

#
# Testing
#

#IF (UNIX)
#  ADD_DEFINITIONS(-fprofile-arcs -ftest-coverage)
#  LINK_LIBRARIES(gcov)
#ENDIF (UNIX)

#ENABLE_TESTING()

IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)

ADD_TEST(testForward ${TEST_PATH}/testForward)

SET_TARGET_PROPERTIES(testForward PROPERTIES PROJECT_LABEL "Tests - testForward")
