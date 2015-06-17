include_directories(${OpenSim_SOURCE_DIR} 
            ${OpenSim_SOURCE_DIR}/Vendors)


link_libraries( debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        )

add_custom_target(copyCMCTestFiles)

if(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
else(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH .)
endif(EXECUTABLE_OUTPUT_PATH)

file(GLOB TEST_PROGS "test*.cpp")

foreach(TEST_PROG ${TEST_PROGS})

    get_filename_component(TEST_ROOT ${TEST_PROG} NAME_WE)

    add_executable(${TEST_ROOT} ${TEST_PROG})
    target_link_libraries(${TEST_ROOT} ${LINK_LIBRARIES})
    add_test(${TEST_ROOT} ${TEST_PATH}/${TEST_ROOT})
    set_target_properties(${TEST_ROOT}
        PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD}
        PROJECT_LABEL "Tests - ${TEST_ROOT}")

endforeach()

file(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

#
# Copy files to run dir
#
add_dependencies(${TEST_ROOT} copyCMCTestFiles)
foreach(dataFile ${TEST_FILES})
 add_custom_command(
    TARGET copyCMCTestFiles
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/Applications/CMC/test)
 endforeach(dataFile) 

#
# Testing
#

#if(UNIX)
#  add_definitions(-fprofile-arcs -ftest-coverage)
#  link_libraries(gcov)
#endif(UNIX)

#enable_testing()
