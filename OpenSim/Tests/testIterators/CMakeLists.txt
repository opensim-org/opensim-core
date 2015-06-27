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

OpenSimCopySharedTestFiles(arm26.osim)

#
# Testing
#
if(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
else(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH .)
endif(EXECUTABLE_OUTPUT_PATH)

add_test(${TEST_TARGET} ${TEST_PATH}/${TEST_TARGET})

