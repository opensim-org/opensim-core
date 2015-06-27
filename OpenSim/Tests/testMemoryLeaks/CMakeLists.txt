file(GLOB SOURCE_FILES *.cpp)
file(GLOB INCLUDE_FILES *.h)

link_libraries(psapi osimSimulation)

add_executable(testMemoryLeaks ${SOURCE_FILES} ${INCLUDE_FILES})

set_target_properties(testMemoryLeaks PROPERTIES
    ${EXCLUDE_IF_MINIMAL_BUILD}
    PROJECT_LABEL "Tests - testMemoryLeaks"
)
