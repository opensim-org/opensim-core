include_directories(${OpenSim_SOURCE_DIR} 
            ${OpenSim_SOURCE_DIR}/Vendors)


link_libraries( debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
        debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        ${Simbody_LIBRARIES}
        )

add_executable(testIK testIK.cpp)

file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
 add_custom_command(
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

add_test(testIK ${TEST_PATH}/testIK)

set_target_properties(testIK PROPERTIES PROJECT_LABEL "Tests - testIK")
