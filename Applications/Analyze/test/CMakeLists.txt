include_directories(${OpenSim_SOURCE_DIR} 
            ${OpenSim_SOURCE_DIR}/Vendors)


link_libraries(
    debug osimSimulation${CMAKE_DEBUG_POSTFIX}   optimized osimSimulation
    debug osimActuators${CMAKE_DEBUG_POSTFIX}    optimized osimActuators
    debug osimCommon${CMAKE_DEBUG_POSTFIX}   optimized osimCommon
    debug osimAnalyses${CMAKE_DEBUG_POSTFIX}     optimized osimAnalyses
    debug osimTools${CMAKE_DEBUG_POSTFIX}        optimized osimTools
    ${Simbody_LIBRARIES}
)

file(GLOB TEST_PROGS "test*.cpp")

foreach(TEST_PROG ${TEST_PROGS})
    get_filename_component(TEST_ROOT ${TEST_PROG} NAME_WE)

    # Link with shared library
    add_executable(${TEST_ROOT} ${TEST_PROG})
    target_link_libraries(${TEST_ROOT} ${LINK_LIBRARIES})
    add_test(${TEST_ROOT} ${EXECUTABLE_OUTPUT_PATH}/${TEST_ROOT})
    set_target_properties(${TEST_ROOT} 
        PROPERTIES 
        PROJECT_LABEL "Tests - ${TEST_ROOT}")

endforeach(TEST_PROG ${TEST_PROGS})

file(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

#
# Copy files to run dir
#
add_dependencies(${TEST_ROOT} copyTestFiles)
foreach(dataFile ${TEST_FILES})
 add_custom_command(
    TARGET testInducedAccelerations
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/Applications/Analyze/test)
 endforeach(dataFile) 

#
# Testing
#

#if(UNIX)
#  add_definitions(-fprofile-arcs -ftest-coverage)
#  link_libraries(gcov)
#endif(UNIX)

#enable_testing()

if(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
else(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH .)
endif(EXECUTABLE_OUTPUT_PATH)
