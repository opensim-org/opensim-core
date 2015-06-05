
LINK_LIBRARIES( 
        debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
        ${Simbody_LIBRARIES})

ADD_EXECUTABLE(testREADME testREADME.cpp)

TARGET_LINK_LIBRARIES(testREADME ${LINK_LIBRARIES} )

FILE(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
    ADD_CUSTOM_COMMAND(
        TARGET testREADME
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy
        ${dataFile}
        ${OpenSim_BINARY_DIR}/OpenSim/Tests/README)
ENDFOREACH (dataFile)


#
# Testing
#

IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)

ADD_TEST(testREADME ${TEST_PATH}/testREADME)

SET_TARGET_PROPERTIES(testREADME PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testREADME")
