INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors)


LINK_LIBRARIES( debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
		debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
		debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
   		debug ${NameSpace}SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized   ${NameSpace}SimTKcommon
   		debug ${NameSpace}SimTKmath${CMAKE_DEBUG_POSTFIX} optimized  ${NameSpace}SimTKmath)

ADD_EXECUTABLE(testCMC testCMC.cpp)
ADD_EXECUTABLE(testCMCWithControlConstraints testCMCWithControlConstraints.cpp)

FILE(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET testCMC
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/Applications/CMC/test)
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


ADD_TEST(testCMC ${TEST_PATH}/testCMC)
ADD_TEST(testCMCWithControlConstraints ${TEST_PATH}/testCMCWithControlConstraints)

SET_TARGET_PROPERTIES(testCMC PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testCMC")
SET_TARGET_PROPERTIES(testCMCWithControlConstraints PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testCMCWithControlConstraints")
