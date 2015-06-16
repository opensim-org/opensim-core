
# Automatically create tests using files named "test*.cpp"  // Adopted from SimTK core
file(GLOB TEST_PROGS "*est*.cpp")

link_libraries(debug osimCommon${CMAKE_DEBUG_POSTFIX}   optimized osimCommon
               ${Simbody_LIBRARIES}
)

foreach(TEST_PROG ${TEST_PROGS})
    get_filename_component(TEST_ROOT ${TEST_PROG} NAME_WE)

    # Link with shared library
    add_executable(${TEST_ROOT} ${TEST_PROG})
    target_link_libraries(${TEST_ROOT} ${LINK_LIBRARIES} )
    add_test(${TEST_ROOT} ${EXECUTABLE_OUTPUT_PATH}/${TEST_ROOT})
    set_target_properties(${TEST_ROOT} 
        PROPERTIES 
        PROJECT_LABEL "Tests - ${TEST_ROOT}")
    # message( "Adding test "${TEST_PROG} )

endforeach(TEST_PROG ${TEST_PROGS})

# Also need to copy data files (.osim, .xml, .sto, .trc, .mot) to run directory
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
 add_custom_command(
    TARGET testSerialization
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/OpenSim/Common/Test)
    # message( "IN file "${dataFile} )
 ENDFOREACH (dataFile) 

