
file(GLOB TEST_PROGS "*est*.cpp")

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS osimLepton
    )
