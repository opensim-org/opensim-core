
# Automatically create tests using files named "test*.cpp"	// Adopted from SimTK core
FILE(GLOB TEST_PROGS "*est*.cpp")

LINK_LIBRARIES(debug osimLepton_d	optimized osimLepton)

FOREACH(TEST_PROG ${TEST_PROGS})
    GET_FILENAME_COMPONENT(TEST_ROOT ${TEST_PROG} NAME_WE)

    # Link with shared library
    ADD_EXECUTABLE(${TEST_ROOT} ${TEST_PROG})
    TARGET_LINK_LIBRARIES(${TEST_ROOT} ${LINK_LIBRARIES})
    ADD_TEST(${TEST_ROOT} ${EXECUTABLE_OUTPUT_PATH}/${TEST_ROOT})
    SET_TARGET_PROPERTIES(${TEST_ROOT} 
        PROPERTIES 
        PROJECT_LABEL "Tests - ${TEST_ROOT}")
    # MESSAGE( "Adding test "${TEST_PROG} )

ENDFOREACH(TEST_PROG ${TEST_PROGS})
