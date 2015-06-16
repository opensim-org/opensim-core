include_directories(${OpenSim_SOURCE_DIR} 
            ${OpenSim_SOURCE_DIR}/Vendors)


link_libraries( debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        ${Simbody_LIBRARIES}
        )

add_executable(testForward testForward.cpp)

file(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
 add_custom_command(
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
#  add_definitions(-fprofile-arcs -ftest-coverage)
#  link_libraries(gcov)
#ENDIF (UNIX)

#enable_testing()

IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)

add_test(testForward ${TEST_PATH}/testForward)

set_target_properties(testForward PROPERTIES PROJECT_LABEL "Tests - testForward")
