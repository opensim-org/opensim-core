file(GLOB TEST_PROGS "test*.cpp")
OpenSimCopySharedTestFiles(gait10dof18musc_subject01.osim)

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS osimTools
    )
