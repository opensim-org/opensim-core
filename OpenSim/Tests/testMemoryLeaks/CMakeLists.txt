
file(GLOB TEST_PROGS "test*.cpp")

OPENSIM_ADD_TESTS(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS psapi osimSimulation
    )
