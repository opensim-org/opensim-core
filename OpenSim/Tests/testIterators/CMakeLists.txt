file(GLOB SOURCE_FILES *.cpp)
file(GLOB INCLUDE_FILES *.h)

link_libraries(osimSimulation osimActuators)

add_executable(testIterators ${SOURCE_FILES} ${INCLUDE_FILES})

set_target_properties(testIterators PROPERTIES
    ${EXCLUDE_IF_MINIMAL_BUILD}
    PROJECT_LABEL "Tests - testIterators"
)
set(TEST_TARGET testIterators)
set(TEST_DIR ${OpenSim_BINARY_DIR}/OpenSim/Tests/testIterators)
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

add_test(${TEST_TARGET} ${TEST_PATH}/${TEST_TARGET})

