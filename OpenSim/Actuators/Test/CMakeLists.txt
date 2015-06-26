
file(GLOB TEST_PROGS "test*.cpp")
OPENSIM_ADD_TESTS(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS osimTools
    )

#if(UNIX)
#  add_definitions(-fprofile-arcs -ftest-coverage)
#  link_libraries(gcov)
#endif(UNIX)
