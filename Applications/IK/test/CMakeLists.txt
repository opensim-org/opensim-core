INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
            ${OpenSim_SOURCE_DIR}/Vendors)


LINK_LIBRARIES( debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
        debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        ${SIMTK_ALL_LIBS}
        )

ADD_EXECUTABLE(testIK testIK.cpp)

FILE(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET testIK
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/Applications/IK/test)
 ENDFOREACH (dataFile) 


IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)

ADD_TEST(testIK ${TEST_PATH}/testIK)

SET_TARGET_PROPERTIES(testIK PROPERTIES PROJECT_LABEL "Tests - testIK")
