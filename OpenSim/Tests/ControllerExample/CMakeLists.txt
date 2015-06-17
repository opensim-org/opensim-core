
link_libraries( debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
        ${Simbody_LIBRARIES})

# Variable definitions
set(EXAMPLE_TARGET exampleController)
set(EXAMPLE_DIR ${OpenSim_SOURCE_DIR}/OpenSim/Examples/ControllerExample)
set(TEST_TARGET testControllerExample)
set(TEST_DIR ${OpenSim_BINARY_DIR}/OpenSim/Tests/ControllerExample)

file(GLOB SOURCE_FILES ${EXAMPLE_DIR}/*.h ${EXAMPLE_DIR}/*.cpp)
add_executable(${EXAMPLE_TARGET} ${SOURCE_FILES})
add_executable(${TEST_TARGET} ${TEST_TARGET}.cpp)

target_link_libraries(${TEST_TARGET} ${LINK_LIBRARIES})

file(GLOB TEST_FILES 
    ${EXAMPLE_DIR}/*.obj 
    ${EXAMPLE_DIR}/*.osim 
    ${EXAMPLE_DIR}/*.xml 
    ${EXAMPLE_DIR}/*.sto 
    ${EXAMPLE_DIR}/*.mot
    *.obj *.osim *.xml *.sto *.mot)

#
# Copy files to run dir
#
foreach(dataFile ${TEST_FILES})
 add_custom_command(
    TARGET ${TEST_TARGET}
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${TEST_DIR})
endforeach(dataFile) 

#
# Testing
#
if(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
else(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH .)
endif(EXECUTABLE_OUTPUT_PATH)

add_test(${TEST_TARGET}Runs ${TEST_PATH}/${EXAMPLE_TARGET})
add_test(${TEST_TARGET} ${TEST_PATH}/${TEST_TARGET})

set_tests_properties(${TEST_TARGET} PROPERTIES DEPENDS ${TEST_TARGET}Runs)

set_target_properties(${EXAMPLE_TARGET} PROPERTIES PROJECT_LABEL "Examples - ${EXAMPLE_TARGET}" )
set_target_properties(${TEST_TARGET} PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - ${TEST_TARGET}")
