
link_libraries( 
        debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
        ${Simbody_LIBRARIES})

add_executable(testComponents testComponents.cpp)

target_link_libraries(testComponents ${LINK_LIBRARIES} )

file(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
    add_custom_command(
        TARGET testComponents
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy
        ${dataFile}
        ${OpenSim_BINARY_DIR}/OpenSim/Tests/Components)
ENDFOREACH (dataFile)


#
# Testing
#

IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)

add_test(testComponents ${TEST_PATH}/testComponents)

set_target_properties(testComponents PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testComponents")
