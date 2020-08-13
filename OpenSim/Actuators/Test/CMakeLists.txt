
file(GLOB TEST_PROGS "test*.cpp")
OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS osimTools osimMoco
    )

OpenSimCopySharedTestFiles(hanging_muscle_CMC_Tasks.xml)


